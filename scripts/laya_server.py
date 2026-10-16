"""Real Laya inference over a bounded JSON API; no ROS installation required."""

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/my_bot'))
from my_bot.laya_protocol import ACTIONS, MODEL_ID, MODEL_REVISION, QUESTIONS, describe_state


class Engine:
    def __init__(self, model, revision, device, threads):
        import torch
        import laya
        torch.set_num_threads(threads)
        self.agent = laya.load(model, revision=revision, device=device)
        self.model = model
        self.revision = self.agent.revision or revision
        self.device = str(self.agent.device)
        self.sdk = laya.__version__
        # Compile/load work must finish before the network listener becomes ready.
        self.decide({'camera_fresh': True, 'lidar_fresh': True,
                     'person_visible': False, 'front_clear': True})

    def decide(self, state):
        started = time.perf_counter()
        answer = self.agent.predict(describe_state(state), QUESTIONS)['answers']['motion']
        action = answer['choice']
        if action not in ACTIONS:
            raise ValueError('Unexpected model action')
        # Laya's `confidence` is entropy based. Gate by the chosen label's
        # probability explicitly, rather than treating entropy as a probability.
        return {'backend': 'laya', 'model_id': self.model, 'model_revision': self.revision,
                'sdk_version': self.sdk, 'device': self.device, 'action': action,
                'confidence': answer['probabilities'][action],
                'probabilities': answer['probabilities'],
                'inference_ms': round((time.perf_counter() - started) * 1000, 2)}


def make_server(address, engine, token=''):
    capacity = threading.BoundedSemaphore(1)

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(5.0)

        def send(self, status, data):
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def authorized(self):
            return not token or hmac.compare_digest(
                self.headers.get('Authorization', ''), 'Bearer ' + token)

        def do_GET(self):
            if not self.authorized():
                self.send(401, {'error': 'Unauthorized'})
            elif self.path == '/health':
                self.send(200, {'ready': True, 'backend': 'laya', 'model_id': engine.model,
                                'revision': engine.revision, 'device': engine.device})
            else:
                self.send(404, {'error': 'Unknown endpoint'})

        def do_POST(self):
            if not self.authorized():
                self.send(401, {'error': 'Unauthorized'})
                return
            if self.path != '/decide':
                self.send(404, {'error': 'Unknown endpoint'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 16384:
                    self.send(413, {'error': 'Body must be 1..16384 bytes'})
                    return
                request = json.loads(self.rfile.read(size))
                request_id = request['request_id']
                if (request['schema_version'] != 1 or not isinstance(request_id, str)
                        or not 1 <= len(request_id) <= 64):
                    raise ValueError('Invalid schema or request id')
                state = request['state']
                describe_state(state)
            except (KeyError, ValueError, TypeError, TimeoutError):
                self.send(400, {'error': 'Invalid request'})
                return
            if not capacity.acquire(blocking=False):
                self.send(503, {'error': 'Inference busy; retry with a fresh observation'})
                return
            try:
                result = engine.decide(state)
                self.send(200, dict(result, schema_version=1, request_id=request_id))
            except Exception as error:
                print('Inference error:', type(error).__name__, flush=True)
                self.send(500, {'error': 'Inference failed'})
            finally:
                capacity.release()

        def log_message(self, _format, *_args):
            pass  # No credentials, raw observations, or request bodies in access logs.

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--model', default=MODEL_ID)
    parser.add_argument('--revision', default=MODEL_REVISION)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--cpu-threads', type=int, default=2)
    args = parser.parse_args()
    token = os.environ.get('LAYA_API_TOKEN', '')
    if args.host not in ('127.0.0.1', 'localhost', '::1') and not token:
        parser.error('Set LAYA_API_TOKEN when exposing the server outside loopback')
    if args.cpu_threads < 1:
        parser.error('--cpu-threads must be positive')
    os.environ.setdefault('HF_HOME', str(ROOT / '.cache/huggingface'))
    os.environ.setdefault('HF_HUB_DISABLE_XET', '1')
    engine = Engine(args.model, args.revision, args.device, args.cpu_threads)
    server = make_server((args.host, args.port), engine, token)
    print(f'Laya ready: {engine.model}@{engine.revision}; {engine.device}; '
          f'http://{args.host}:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
