qqimport numpy as np
import cv2
import time

def detect(ShapeReelMeter):
    # Load Yolo
    net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))
    cap = cv2.VideoCapture(0)

    # Capture frame-by-frame
    ret, img = cap.read()
    
    # Our operations on the frame come here
    # img = cv2.resize(img, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape
    
    
    # Detecting objects
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)
    
    # vectors
    Vx = 0
    Vy = 0
    
    # Object detection
    isDetected = false
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    # print(indexes)
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            # Agi egittikten sonra kalkacak
            if "cell phone" == str(classes[class_ids[i]]):
            isDetected = true
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
                cv2.line(img, (width/2, height/2), (x + w/2, y + h/2), color, 4)
                Vx = (x + w/2) - width/2
                Vy = (y + h/2) - height/2
            # print(label)
        
    cap.release()
    return Vx*(ShapeReelMeter/w), Vy*(ShapeReelMeter/h)
    
if __name__ == "__main__":
    
    v1, v2 = detect(1)
    print v1, v2


    # When everything done, release the capture
    # cap.release()
    # cv2.destroyAllWindows()