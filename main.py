from camera.worker import Worker

worker = Worker()
try:
    worker.track_target()
except KeyboardInterrupt:
    worker.shutdown()
