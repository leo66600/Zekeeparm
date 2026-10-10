"""Local plan/confirm/execute boundary. Chat and MCP never execute a plan."""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
import secrets
import threading
import time
from .web_network import llm_network

from .planner import MUTATING_TOOLS, READ_ONLY_TOOLS, LEASED_TOOLS, TOOL_SCHEMAS, validate_arguments, parse_plan, plan_as_dict


class AgentSession:
    def __init__(self, planner, robot, *, motion_authorized=False, lease=None, clock=time.monotonic):
        self.planner, self.robot, self.lease = planner, robot, lease
        self.motion_authorized, self.clock = motion_authorized, clock
        self.lock = threading.Lock()
        self.plans = {}
        self.approvals = {}
        self.approval_lock = threading.Lock()
        self.events = deque(maxlen=200)
        self.cancelled = threading.Event()
        self.active = False
        self.persistent_owner = False
        self.blocked = False
        self.heartbeat = clock()

    def event(self, kind, **values):
        event = {'type': kind, 'time': time.time(), **values}
        self.events.append(event)
        return event

    def status(self):
        self.heartbeat = self.clock()
        return {'ok': True, 'mode': 'plan-review-v1', 'active': self.active or self.persistent_owner, 'blocked': self.blocked,
                'motion_authorized': self.motion_authorized, 'planner_configured': self.planner is not None, 'recording': getattr(self.robot,'_recording',False), 'recorded_samples': len(getattr(self.robot,'_recorded',[])), 'events': list(self.events)}

    def plan(self, text):
        if self.planner is None:
            raise RuntimeError('missing ZKEEP_LLM_API_KEY; set it in backend environment and restart web_ai')
        if not isinstance(text, str) or not 1 <= len(text.strip()) <= 1000:
            raise ValueError('text must contain 1–1000 characters')
        # Re-parse at both boundaries, including injected/offline planners.
        plan = parse_plan(json.dumps(plan_as_dict(self.planner.plan(text))))
        with self.lock:
            now = self.clock()
            self.plans = {key: value for key, value in self.plans.items() if now-value[1] < 120}
            if len(self.plans) >= 32:
                raise ValueError('too many pending plans')
            token = secrets.token_urlsafe(24)
            self.plans[token] = (plan, now)
        return {'ok': True, 'text': plan.summary, 'plan': plan_as_dict(plan),
                'plan_id': token, 'expires_in': 120, 'requires_confirmation': plan.requires_confirmation}

    def execute(self, token, confirmed):
        if confirmed is not True:
            raise ValueError('explicit confirmation required')
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('AI execution busy')
        acquired = self.persistent_owner
        started = False
        execution_events = []
        try:
            if self.blocked:
                raise RuntimeError('previous stop/release unconfirmed; restart only after controller inspection')
            stored = self.plans.pop(token, None)
            if stored is None or self.clock()-stored[1] >= 120:
                raise ValueError('plan expired, unknown or already consumed')
            plan = parse_plan(json.dumps(plan_as_dict(stored[0])))
            if plan.requires_confirmation and not self.motion_authorized:
                raise PermissionError('backend motion authorization is off')
            self.cancelled.clear()
            self.active = True
            self.heartbeat = self.clock()
            for step in plan.steps:
                if self.cancelled.is_set():
                    raise RuntimeError('execution cancelled')
                prepare = getattr(self.robot, 'prepare', None)
                if prepare is not None:
                    prepare(step.tool)
                if self.cancelled.is_set():
                    raise RuntimeError('execution cancelled during backend startup')
                if step.tool in LEASED_TOOLS and self.lease and not acquired:
                    self.lease(True)
                    acquired = True
                if acquired and step.tool in ('pick_color','place_object'):
                    if self.persistent_owner:
                        raise RuntimeError('先停止重力补偿，再进行颜色抓取或放置')
                    self.lease(False)
                    acquired = False
                if step.tool in MUTATING_TOOLS:
                    started = True
                    grant = secrets.token_urlsafe(24)
                    with self.approval_lock:
                        self.approvals[grant] = (step.tool, step.arguments, self.clock())
                    response = self.rpc({'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                                         'params': {'name': step.tool, 'arguments': {**step.arguments, 'approval_token': grant}}})
                    result = response['result']['content'][0]['text']
                    if response['result'].get('isError'):
                        raise RuntimeError(result)
                else:
                    result = self.robot.execute(step.tool, step.arguments)
                if self.cancelled.is_set():
                    raise RuntimeError('execution cancelled; hold required')
                if step.tool == 'gravity_compensation_start':self.persistent_owner = True
                if step.tool in ('stop','gravity_compensation_stop','disable_robot'):self.persistent_owner = False
                execution_events.append(self.event('tool', name=step.tool, arguments=step.arguments, result=result))
            return {'ok': True, 'text': '计划执行完成', 'events': list(self.events), 'execution_events': execution_events}
        except Exception as exc:
            self.event('error', message=str(exc))
            if getattr(self.robot,'hold_unconfirmed',False):self.blocked=True
            if self.active and (acquired or (started and self.lease is None)):
                try:
                    # Unlike stop_best_effort, success must be observed.
                    self.robot.execute('stop')
                    self.persistent_owner = False
                except Exception:
                    self.blocked = True
            raise
        finally:
            if acquired and not self.blocked and not self.persistent_owner:
                try:
                    self.lease(False)
                except Exception:
                    self.blocked = True
                    self.event('error', message='controller lease release unconfirmed')
            with self.approval_lock:
                self.approvals.clear()
            self.active = False
            self.lock.release()
            if self.blocked:
                raise RuntimeError('controller hold/lease release unconfirmed')

    def cancel(self, wait=False):
        self.cancelled.set()
        if self.persistent_owner and not self.active and self.lock.acquire(blocking=False):
            try:
                self.robot.execute('stop')
                if self.lease:self.lease(False)
                self.persistent_owner = False
            except Exception:
                self.blocked = True
                raise
            finally:self.lock.release()
        if wait:
            deadline = time.monotonic()+8
            while self.active and time.monotonic()<deadline:
                time.sleep(.02)
            if self.active or self.persistent_owner or self.blocked:
                raise RuntimeError('AI task termination/hold unconfirmed')
        return {'ok': True, 'success': not self.active and not self.persistent_owner and not self.blocked, 'text': 'AI 任务已结束' if not self.active else '取消请求已发送；等待停止结果', 'active': self.active}

    def watchdog(self):
        if self.blocked:return
        if (self.active or self.persistent_owner) and self.clock()-self.heartbeat > 3:
            self.cancel()
        if not self.active and (getattr(self.robot,'_recording',False) or self.persistent_owner) and self.lock.acquire(blocking=False):
            try:self.robot.poll()
            except Exception as exc:self.event('error',message=str(exc))
            finally:self.lock.release()

    def rpc(self, request):
        if not isinstance(request, dict) or request.get('jsonrpc') != '2.0':
            raise ValueError('JSON-RPC 2.0 required')
        method, params = request.get('method'), request.get('params', {})
        if method == 'initialize':
            result = {'protocolVersion': '2025-03-26', 'capabilities': {'tools': {}},
                      'serverInfo': {'name': 'zekeep-plan-review', 'version': '1.0.0'}}
        elif method == 'ping' and params == {}:
            result = {}
        elif method == 'notifications/initialized':
            return None
        elif method == 'tools/list':
            tools=[{'name':'plan','description':'文本生成计划；不执行。','inputSchema':{
                'type':'object','properties':{'text':{'type':'string','maxLength':1000}},'required':['text'],'additionalProperties':False}}]
            for name,contract in TOOL_SCHEMAS.items():
                contract=json.loads(json.dumps(contract))
                if name in MUTATING_TOOLS:
                    contract['properties']['approval_token']={'type':'string'}
                    contract['required'].append('approval_token')
                tools.append({'name':name,'description':name+('；需人工确认后一次性授权' if name in MUTATING_TOOLS else '；只读'), 'inputSchema':contract})
            result={'tools':tools}
        elif method == 'tools/call':
            args = params.get('arguments', {})
            if params.get('name') == 'plan' and isinstance(args, dict) and set(args) == {'text'}:
                value = self.plan(args['text'])
                # Review token stays on the human web surface, not in model tool output.
                value.pop('plan_id')
            elif params.get('name') in READ_ONLY_TOOLS:
                tool=params['name'];validate_arguments(tool,args)
                if not self.lock.acquire(blocking=False):
                    raise RuntimeError('AI task busy; diagnostic query deferred')
                try:value=self.robot.execute(tool,args)
                finally:self.lock.release()
            elif params.get('name') in MUTATING_TOOLS and isinstance(args,dict) and 'approval_token' in args:
                try:
                    token=args['approval_token'];arguments={key:value for key,value in args.items() if key!='approval_token'}
                    validate_arguments(params['name'],arguments)
                    if not isinstance(token,str):raise PermissionError('human confirmation grant required')
                    with self.approval_lock:approved=self.approvals.pop(token,None)
                    if not self.active or not self.motion_authorized or not approved or approved[0]!=params['name'] or approved[1]!=arguments or self.clock()-approved[2]>60:
                        raise PermissionError('human confirmation grant expired, altered or unavailable')
                    if self.cancelled.is_set():raise RuntimeError('execution cancelled')
                    value=self.robot.execute(params['name'],arguments)
                except Exception as exc:
                    return {'jsonrpc':'2.0','id':request.get('id'),'result':{
                        'isError':True,'content':[{'type':'text','text':str(exc)}]}}
            else:
                raise ValueError('tool or arguments not allowed; motion requires human confirmation grant')
            result = {'content': [{'type': 'text', 'text': value if isinstance(value, str) else json.dumps(value, ensure_ascii=False)}]}
        else:
            raise ValueError('method not supported')
        return {'jsonrpc': '2.0', 'id': request.get('id'), 'result': result}


def serve(session, port=8082):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, value):
            raw = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path in ('/health', '/status'):
                self.reply(200, session.status())
            elif self.path == '/mcp':
                self.reply(405, {'error': 'SSE not supported'})
            else:
                self.reply(404, {'ok': False, 'error': 'unknown endpoint'})

        def do_POST(self):
            try:
                # This loopback endpoint accepts only the local Node proxy/CLI.
                if self.headers.get('Origin') or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise PermissionError('local JSON proxy required')
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 16384:
                    raise ValueError('body must be 1–16384 bytes')
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError('JSON object required')
                if self.path in ('/chat', '/plan') and set(body) == {'text'}:
                    result = session.plan(body['text'])
                elif self.path == '/execute' and set(body) == {'plan_id', 'confirmed'}:
                    result = session.execute(body['plan_id'], body['confirmed'])
                    if session.blocked:
                        raise RuntimeError('stop/release unconfirmed')
                elif self.path == '/cancel' and body == {}:
                    result = session.cancel(wait=True)
                elif self.path == '/mcp':
                    result = session.rpc(body)
                    if result is None:
                        self.send_response(202)
                        self.send_header('Content-Length', '0')
                        self.end_headers()
                        return
                else:
                    raise ValueError('unknown endpoint or extra fields')
                self.reply(200, result)
            except Exception as exc:
                if self.path == '/mcp':
                    self.reply(200, {'jsonrpc': '2.0', 'id': body.get('id') if isinstance(locals().get('body'), dict) else None,
                                     'error': {'code': -32602, 'message': str(exc)}})
                else:
                    self.reply(403 if isinstance(exc, PermissionError) else 400, {'ok': False, 'error': str(exc)})

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    def watchdog():
        while True:
            time.sleep(.05)
            try:session.watchdog()
            except Exception as exc:session.event('error',message=str(exc))
    threading.Thread(target=watchdog, daemon=True).start()
    return server


@llm_network()
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--motion-authorized', action='store_true')
    args = parser.parse_args()
    from .deepseek import DeepSeekPlanner
    from .robot import RobotTools
    import rclpy
    from rclpy.node import Node
    from zekeep_msgs.srv import WebTaskLease
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def shutdown_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown_signal)
    import os
    planner = DeepSeekPlanner() if (os.environ.get('ZKEEP_LLM_API_KEY') or os.environ.get('ZKEEP_VLM_API_KEY', '')).strip() else None
    cancelled = threading.Event()
    robot = RobotTools(command_namespace='zekeep/web_task', cancel_event=cancelled, autostart=args.motion_authorized)
    lease_node = Node('zekeep_ai_lease')
    from rclpy.executors import SingleThreadedExecutor
    lease_executor = SingleThreadedExecutor(context=lease_node.context)
    lease_executor.add_node(lease_node)
    client = lease_node.create_client(WebTaskLease, '/zekeep/web_task/lease')
    lease_lock, owned = threading.Lock(), threading.Event()
    def lease(acquire, renew=False):
        with lease_lock:
            if renew and not owned.is_set():
                return
            if not client.wait_for_service(timeout_sec=1):
                raise RuntimeError('controller lease unavailable')
            request = WebTaskLease.Request(owner='ai', acquire=acquire, renew=renew)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(lease_node, future, executor=lease_executor, timeout_sec=1)
            if not future.done() or future.result() is None:
                session.blocked = True
                raise RuntimeError('controller lease response unknown; inspect controller ownership')
            if not future.result().success:
                raise RuntimeError(future.result().message or 'controller lease rejected')
            if not renew:
                owned.set() if acquire else owned.clear()
    session = AgentSession(planner, robot, motion_authorized=args.motion_authorized, lease=lease)
    session.cancelled = cancelled
    def renew():
        while rclpy.ok():
            time.sleep(.5)
            if session.blocked:
                owned.clear()
                cancelled.set()
            if owned.is_set():
                try:
                    lease(True, True)
                except Exception:
                    cancelled.set()
                    owned.clear()  # Controller watchdog holds; no further task command.
                    session.blocked = True
    threading.Thread(target=renew, daemon=True).start()
    server = serve(session, args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        session.cancel()
        deadline = time.monotonic()+8
        while session.active and time.monotonic()<deadline:
            time.sleep(.05)
    finally:
        server.server_close()
        if not session.active:
            robot.destroy_node()
            with lease_lock:
                lease_executor.shutdown()
                lease_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
