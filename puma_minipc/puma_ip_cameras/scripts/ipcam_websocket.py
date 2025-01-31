import websockets
import rospy
import threading
import asyncio
from puma_ip_cameras.utils_video_ip import AsyncVideoCapture
import json

async def send_video_ws_to_back(rtsp_url, backend_url, camera_name):
  video_cam = AsyncVideoCapture(rtsp_url)
  video_cam.start()
  
  try:
    async with websockets.connect(backend_url) as websocket:
      while True:
        msg = json.dumps({
          "name": camera_name,
          "img": video_cam.get_encode_frame_to_jpeg()
        })
        
        await websocket.send(msg)      
        await asyncio.sleep(0.1)
      
  except Exception as e:
    rospy.logerr(f"Error de conexion: {str(e)}")
    # await asyncio.sleep(reconnect_delay)
      
  finally:
    if websocket is not None:
      await websocket.close()

def start_asyncio_task(rtsp_url, backend_url, camera_name):
  asyncio.run(send_video_ws_to_back(rtsp_url, backend_url, camera_name))

if __name__ == "__main__":
  rospy.init_node("websocket_node")
  rtsp_url = "rtsp://admin:smartbot2023@10.42.0.103:554/cam/realmonitor?channel=1&subtype=0"
  backend_url = "ws://10.42.0.22:8000/ipcam/ws"
  camera_name = "robot_camera"
  
  thread = threading.Thread(target=start_asyncio_task, args=(rtsp_url, backend_url, camera_name))
  thread.start()
  
  rospy.loginfo("Nodo de streaming iniciado")
  try:
    rospy.spin()
  
  except Exception as e:
    rospy.logerr(f"Error: {str(e)}")
  
  finally:
    thread.join()