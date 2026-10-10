"""Optional app-owned DeepSeek exit; does not change the system Clash configuration."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile
import time


def network_settings(path):
    value=json.loads(Path(path).read_text())
    if not isinstance(value,dict) or set(value)!={'direct_interface','proxy_port'}:
        raise ValueError('LLM network profile requires direct_interface and proxy_port')
    if not isinstance(value['direct_interface'],str) or not re.fullmatch(r'[A-Za-z0-9_.:-]+',value['direct_interface']):
        raise ValueError('invalid LLM physical interface')
    if type(value['proxy_port']) is not int or not 1024<=value['proxy_port']<=65535:
        raise ValueError('invalid LLM proxy port')
    return value


def listening(port):
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=.1):return True
    except OSError:return False


@contextmanager
def llm_network():
    profile=Path(os.environ.get('ZKEEP_LLM_NETWORK_FILE',str(Path.home()/'.local/share/zekeep/llm_network.json')))
    if not profile.is_file():
        yield
        return
    settings=network_settings(profile)
    port=settings['proxy_port'];interface=settings['direct_interface']
    url=f'http://127.0.0.1:{port}'
    previous=os.environ.get('ZKEEP_LLM_PROXY_URL')
    # An explicitly configured different proxy takes precedence over the saved local exit.
    if previous and previous!=url:
        yield
        return
    process=None
    with tempfile.TemporaryDirectory(prefix='zekeep-llm-exit-') as directory:
        try:
            if not listening(port):
                config=Path(directory)/'config.yaml'
                config.write_text(f'''mixed-port: {port}
bind-address: 127.0.0.1
allow-lan: false
mode: rule
interface-name: {interface}
log-level: warning
dns:
  enable: true
  enhanced-mode: redir-host
  nameserver: [223.5.5.5, 119.29.29.29]
rules:
  - DOMAIN,api.deepseek.com,DIRECT
  - MATCH,REJECT
''')
                process=subprocess.Popen(['/usr/bin/verge-mihomo','-d',directory,'-f',str(config)])
                deadline=time.monotonic()+5
                while not listening(port):
                    if process.poll() is not None or time.monotonic()>deadline:
                        raise RuntimeError('LLM dedicated exit failed to start; check Mihomo/interface')
                    time.sleep(.05)
            os.environ['ZKEEP_LLM_PROXY_URL']=url
            yield
        finally:
            if previous is None:os.environ.pop('ZKEEP_LLM_PROXY_URL',None)
            else:os.environ['ZKEEP_LLM_PROXY_URL']=previous
            if process is not None and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=3)
