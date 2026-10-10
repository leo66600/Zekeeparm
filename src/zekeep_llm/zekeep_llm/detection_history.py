"""Short observation history for text queries, never motion coordinates."""
import math

WINDOW_S = 2.0


def recent_observations(frames, now_ros, identity):
    tracks = []
    seen_stamps = set()
    for body in frames:
        stamp = body.get('stamp', {})
        captured = stamp.get('sec', 0) + stamp.get('nanosec', 0) / 1e9
        if not 0 <= now_ros-captured <= WINDOW_S or captured in seen_stamps:
            continue
        if body.get('calibration_status') != 'CALIBRATED_STATIONARY' or body.get('calibration_identity') != identity:
            continue
        seen_stamps.add(captured)
        used = set()
        for item in body.get('visual_detections', body.get('detections', [])):
            box = item.get('bbox', [])
            name = item.get('class_name', item.get('color'))
            confidence = item.get('confidence')
            if (len(box) != 4 or not all(type(v) in (int, float) and math.isfinite(v) for v in box)
                    or box[2] <= box[0] or box[3] <= box[1]
                    or not isinstance(name, str) or type(confidence) not in (int, float)
                    or not math.isfinite(confidence) or not 0 <= confidence <= 1):
                continue
            best, score = None, .5
            for index, track in enumerate(tracks):
                if index in used or track['class_name'] != name:
                    continue
                other = track['bbox']
                intersection = max(0, min(box[2], other[2])-max(box[0], other[0])) * max(0, min(box[3], other[3])-max(box[1], other[1]))
                union = (box[2]-box[0])*(box[3]-box[1]) + (other[2]-other[0])*(other[3]-other[1]) - intersection
                if intersection/union > score:
                    best, score = index, intersection/union
            if best is None:
                best = len(tracks)
                tracks.append(dict(class_name=name, confidence=confidence, bbox=box, hits=0))
            track = tracks[best]
            track.update(bbox=box, confidence=confidence, hits=track['hits']+1)
            used.add(best)
    return [track for track in tracks if track['hits'] >= 2]
