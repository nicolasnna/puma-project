#/usr/bin/env python3
import rospy
import threading
import asyncio
import json
import websockets

async def send_msg_to_backend(backend_url):
  try:
    async with websockets.connect(backend_url) as websocket:
      while True:
        await websocket.send(json.dumps({"msg": "Hello from ROS"}))
        await asyncio.sleep(1)
  
  except Exception as e:
    rospy.logerr(f"Error de conexion: {str(e)}")
    await websocket.close()
    
def start_asyncio_task(backend_url):
  asyncio.run(send_msg_to_backend( backend_url))    

if __name__ == '__main__':
    rospy.init_node('ros_to_websocket')
    rospy.loginfo('ROS to WebSocket node started')
    backend_url = "ws://smartbot.la/test2api/ros/ws/puma"
    thread = threading.Thread(target=start_asyncio_task, args=(backend_url,))
    thread.start()

    rospy.loginfo("Nodo de streaming ROS iniciado")
    try:
      rospy.spin()
    except:
      rospy.loginfo('ROS to WebSocket node stopped')
      thread.join()