import asyncio
import websockets
import base64
import cv2
import numpy as np
import yaml
import time

class VideoClient:
    def __init__(self, server_url):
        self.server_url = server_url
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((4 * 6, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        self.found = 0

    async def video_client(self):
        print("Connected to server")
        try:
            async with websockets.connect(self.server_url) as websocket:
                while True:
                        frame_data = await websocket.recv()
                        
                        frame_data = base64.b64decode(frame_data)
                        frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        ret, corners = cv2.findChessboardCorners(gray, (6, 4), None)
                        if ret == True:
                            self.objpoints.append(self.objp)
                            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                            self.imgpoints.append(corners2)
                            frame = cv2.drawChessboardCorners(frame, (6, 4), corners2, ret)
                            self.found += 1

                        cv2.imshow('Frame', frame)
                        if self.found == 50:
                            cv2.destroyAllWindows()
                            await websocket.close()
                            break
                            
                            
                        if cv2.waitKey(25) & 0xFF == ord('q'):
                            cv2.destroyAllWindows()
                            await websocket.close()
                            break
        except websockets.exceptions.ConnectionClosedError as e:
            print(f"Connection closed unexpectedly: {e}")
        except Exception as e:
            print(f"Error in VideoClient: {e}")
        finally:
            print(f"Connection closed, finnaly")
                            

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, gray.shape[::-1], None, None
            )

        data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
        with open("calibration.yaml", "w") as f:
                yaml.dump(data, f)


    def start(self):
        asyncio.get_event_loop().run_until_complete(self.video_client())

if __name__ == '__main__':
    conexion_obj=VideoClient('ws://omni.local:8765')
    conexion_obj.start()

