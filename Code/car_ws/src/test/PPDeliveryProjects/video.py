import cv2

vc=cv2.VideoCapture(0)  
vc.set(3, 640) # width
vc.set(4, 480) # height

fps=20         
size=(int(vc.get(cv2.CAP_PROP_FRAME_WIDTH)),
      int(vc.get(cv2.CAP_PROP_FRAME_HEIGHT)))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out=cv2.VideoWriter()
out.open("output0413.mp4", fourcc, fps, size)
while True:
    ret, frame  = vc.read()
    if ret==False:
        break
    #frame = cv2.flip(frame,1)
    out.write(frame)
    cv2.imshow("frame",frame)  
    key=cv2.waitKey(25)
    #print(1)
    if key==27:                  
        break
    success,frame=vc.read()       
vc.release() 
out.release() 
cv2.destroyAllWindows()

