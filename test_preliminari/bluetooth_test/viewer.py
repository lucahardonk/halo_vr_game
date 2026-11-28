"""Server diagnostico esclusivamente loopback; nessun accesso BLE dal thread HTTP."""
import secrets
import threading

from flask import Flask, jsonify, render_template, request
from werkzeug.serving import make_server, WSGIRequestHandler


class QuietHandler(WSGIRequestHandler):
    def log_request(self, code='-', size='-'):
        # Non mescolare 20 access log/s con la stampa diagnostica BLE.
        if str(code).startswith(('4', '5')):
            super().log_request(code, size)


def create_app(state):
    app = Flask(__name__)
    token = secrets.token_urlsafe(32)

    @app.after_request
    def no_cache(response):
        response.headers['Cache-Control'] = 'no-store'
        return response

    @app.get('/')
    def index():
        return render_template('index.html', token=token)

    @app.get('/api/state')
    def snapshot():
        return jsonify(state.snapshot())

    @app.post('/api/reset')
    def reset():
        if request.headers.get('X-Reset-Token') != token:
            return jsonify(error='Richiesta non autorizzata'), 403
        if not state.reset():
            return jsonify(error='Orientamento non disponibile'), 409
        return jsonify(ok=True, state=state.snapshot())

    return app


class Viewer:
    def __init__(self, state, port):
        self.server = make_server('127.0.0.1', port, create_app(state), threaded=True, request_handler=QuietHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, name='imu-http')

    def start(self):
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.thread.join(timeout=5)
        self.server.server_close()
