# web_server.py
import http.server
import socketserver
import json
from constants import shared_data, data_lock, CONFIG, WEB_SERVER_PORT

class RequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Establece la ruta por defecto a index.html
        if self.path == '/':
            self.path = '/index.html'
            return http.server.SimpleHTTPRequestHandler.do_GET(self)

        # Endpoint para los datos del mapa
        if self.path == '/mapdata':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with data_lock:
                self.wfile.write(json.dumps(shared_data).encode('utf-8'))
            return

        # Endpoint para la configuración
        if self.path == '/config':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with data_lock:
                # Se envía una copia para evitar problemas de concurrencia si se modifica
                self.wfile.write(json.dumps(CONFIG.copy()).encode('utf-8'))
            return
        
        # Para cualquier otro archivo, usa el manejador por defecto (sirve archivos locales)
        return http.server.SimpleHTTPRequestHandler.do_GET(self)

def start_web_server():
    # Usamos TCPServer para permitir la reutilización de la dirección
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", WEB_SERVER_PORT), RequestHandler) as httpd:
        print(f"Servidor web iniciado en http://localhost:{WEB_SERVER_PORT}")
        httpd.serve_forever()