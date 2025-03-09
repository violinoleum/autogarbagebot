from ultralytics import YOLO

model = YOLO("best_2_24.pt")

results = model("test_sim_far.jpg")

results[0].show()