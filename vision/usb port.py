import cv2 as cv

# Zoek naar beschikbare camera's (tot 5 proberen)
cameras = []
for i in range(5):
    cam = cv.VideoCapture(i)
    if cam.isOpened():
        cameras.append(i)
        cam.release()

if not cameras:
    print("Geen camera's gevonden.")
    exit()

print(f"Gevonden camera's: {cameras}")

# Toon een preview van elke gevonden camera
for cam_index in cameras:
    cam = cv.VideoCapture(cam_index)
    
    if not cam.isOpened():
        print(f"Camera {cam_index} kon niet worden geopend.")
        continue
    
    print(f"Toont camera {cam_index}... (Druk op 'q' om door te gaan)")
    
    while True:
        ret, frame = cam.read()
        if not ret:
            print(f"Kon geen frame vastleggen van camera {cam_index}")
            break
        
        cv.putText(frame, f"Camera {cam_index}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 0), 2, cv.LINE_AA)
        
        cv.imshow(f"Camera {cam_index}", frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.release()
    cv.destroyAllWindows()

print("Alle camera's zijn getest.")
