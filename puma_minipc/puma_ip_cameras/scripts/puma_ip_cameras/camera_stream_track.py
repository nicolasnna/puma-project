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