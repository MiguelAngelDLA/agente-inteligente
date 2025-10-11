# web_server.py
import http.server
import socketserver
import json
from constants import shared_data, data_lock, CONFIG, WEB_SERVER_PORT

class RequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = '/index.html'
            return http.server.SimpleHTTPRequestHandler.do_GET(self)

        if self.path == '/mapdata':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with data_lock:
                self.wfile.write(json.dumps(shared_data).encode('utf-8'))
            return

        if self.path == '/config':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with data_lock:
                self.wfile.write(json.dumps(CONFIG.copy()).encode('utf-8'))
            return
        
        return http.server.SimpleHTTPRequestHandler.do_GET(self)

def start_web_server():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", WEB_SERVER_PORT), RequestHandler) as httpd:
        print(f"Servidor web iniciado en http://localhost:{WEB_SERVER_PORT}")
        httpd.serve_forever()