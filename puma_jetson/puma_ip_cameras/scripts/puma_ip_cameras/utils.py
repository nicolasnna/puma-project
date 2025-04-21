from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
import websockets
import rospy
import json
import asyncio

from aiortc import VideoStreamTrack
from puma_ip_cameras.utils_video_ip import AsyncVideoCapture
from av import VideoFrame
import cv2
import fractions
from datetime import datetime

class CameraStreamTrack(VideoStreamTrack):
  '''  '''
  def __init__(self, rtsp_url):
    super().__init__()
    self.capv = AsyncVideoCapture(rtsp_url)
    self.capv.start()
    self.frame_count = 0
    
  async def recv(self):
    self.frame_count += 1
    print(f"Sending frame {self.frame_count}")
    frame = self.capv.get_frame()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
    video_frame.pts = self.frame_count
    video_frame.time_base = fractions.Fraction(1, 30)  # Use fractions for time_base
    # Add timestamp to the frame
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Current time with milliseconds
    cv2.putText(frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
    video_frame.pts = self.frame_count
    video_frame.time_base = fractions.Fraction(1, 30)  # Use fractions for time_base
    return video_frame

def ice_candidate_from_dict(data):
    """
    Convierte un diccionario con datos de un candidato ICE a un objeto RTCIceCandidate.
    """
    return RTCIceCandidate(
        sdpMid=data["sdpMid"],
        sdpMLineIndex=data["sdpMLineIndex"],
        candidate=data["candidate"]
    )

async def send_video_to_backend(rtsp_url, backend_url, camera_name):  
  # pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]))
  pc = RTCPeerConnection()
  video_sender = CameraStreamTrack(rtsp_url)
  pc.addTrack(video_sender)
  
  try:
    async with websockets.connect(backend_url) as websocket:
      rospy.loginfo("Conexion establecida con el backend")
      
      @pc.on("datachannel")
      def on_datachannel(channel):
        rospy.loginfo(f"Nuevo canal de datos: {channel.label}")
      
      @pc.on("connectionstatechange")
      async def on_connectionstatechange():
        rospy.loginfo(f"Estado de la conexion: {pc.connectionState}")
        if pc.connectionState == "connected":
          rospy.loginfo("Conexion establecida")
          
      offer = await pc.createOffer()
      await pc.setLocalDescription(offer)
      await websocket.send(json.dumps({"name": camera_name, "type": "offer", "sdp": offer.sdp}))
      
      while True:
        # Manejar ICE y respuesta
        message = json.loads(await websocket.recv())
        rospy.loginfo(f"Estado de la conexion: {pc.connectionState}")
        if message is None:
          rospy.logwarn("No se recibieron datos del backend")
          break
        
        if message["type"] == "answer":
          answer = RTCSessionDescription(sdp=message["sdp"], type=message["type"])
          await pc.setRemoteDescription(answer)
          rospy.loginfo("Respuesta configurada como descripcion remota")
          rospy.loginfo(f"local description: {pc.localDescription.sdp}, local description: {pc.localDescription.type}")
          rospy.loginfo(f"remote description: {pc.remoteDescription.sdp}, remote description: {pc.remoteDescription.type}")
        rospy.loginfo(f"Estado de la conexión: {pc.connectionState}")
        await asyncio.sleep(0.2)
        
  except Exception as e:
    rospy.logerr(f"Error de conexion: {str(e)}")
    # await asyncio.sleep(reconnect_delay)
    # rospy.loginfo(f"Reintentando en {reconnect_delay} segundos...")
    # reconnect_delay = min(reconnect_delay * 1.3, 60)
    
  finally:
    websocket.close()