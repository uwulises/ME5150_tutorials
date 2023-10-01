import asyncio
import websockets
import base64
import picamera
import io
import time

class videoServer:
    def __init__(self):
        self.camera= None
        self.frame_count= 0
        self.max_frame_count= 70000000

    async def videoServer(self, websocket, path):
        try:
            print(f"Client connected desde {websocket.remote_address}")
            self.camera= picamera.PiCamera()
            self.camera.resolution= (640, 480)
            self.camera.framerate= 24
            self.camera.start_preview()
            time.sleep(2)
            stream= io.BytesIO()
            for foo in self.camera.capture_continuous(stream, 'jpeg', use_video_port=True):
                stream.seek(0)
                await websocket.send(base64.b64encode(stream.read()))
                stream.seek(0)
                stream.truncate()
                self.frame_count+= 1
                if self.frame_count>self.max_frame_count:
                   self.frame_count=0
                   break
        except Exception as e:
            print(f"Error en videoServer: {e}")
        finally:
            self.camera.close()
            print(f"Client disconnected desde {websocket.remote_address}")

            
    def start(self):
        start_server = websockets.serve(self.videoServer, "0.0.0.0", 8765)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()

if __name__ == '__main__':

    videoServer().start()
