from puma_ip_cameras.camera_stream_track import CameraStreamTrack
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
import websockets
import rospy
import json
import asyncio

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