import threading
import webbrowser
import time
from web_server import start_web_server
from simulation import Game

if __name__ == '__main__':
    server_thread = threading.Thread(target=start_web_server, daemon=True)
    server_thread.start()
    
    time.sleep(1)
    webbrowser.open('http://localhost:8000')

    game = Game()
    game.run()