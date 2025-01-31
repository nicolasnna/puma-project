import queue
import threading
import cv2
import rospy
import base64

class AsyncVideoCapture:
  ''' Clase para capturar video de una cámara IP con asyncio - No bloqueante '''
  def __init__(self, rtsp_url):
    self.rtsp_url = rtsp_url
    self.queue = queue.Queue(maxsize=2)
    self.running = False
    self.thread = None
    
  def start(self):
    self.running = True
    self.thread = threading.Thread(target=self._capture_frames, daemon=True)
    self.thread.start()
    
  def _capture_frames(self):
    cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    
    while self.running:
      try:
        ret, frame = cap.read()
        if not ret:
          rospy.logwarn('Reconectando a la cámara...')
          cap.release()
          cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
          continue
        
        if self.queue.qsize() >= 2:
          self.queue.get_nowait()
        self.queue.put(frame)
      except Exception as e:
        rospy.logerr(f"Error captura: {str(e)}")
        cap.release()
        self.running = False
        
  def get_frame(self):
    return self.queue.get()
    
  def stop(self):
    self.running = False
    if self.thread:
      self.thread.join()
      
  def get_encode_frame_to_jpeg(self):
    frame = self.get_frame()
    ret, jpeg = cv2.imencode('.jpg', frame)
    if not ret:
      rospy.logerr("Error encoding frame to JPEG")
      return None
    jpeg_bytes = jpeg.tobytes()

        # Convierte los bytes a una cadena base64
    jpeg_base64 = base64.b64encode(jpeg_bytes).decode('utf-8')
    
    return jpeg_base64