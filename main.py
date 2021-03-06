from camera.worker import Worker

worker = Worker()
try:
    worker.start_hunting()
except KeyboardInterrupt:
    worker.shutdown()
