import RPi.GPIO as GPIO
from time import sleep
import cv2
import numpy as np

in1 = 24
in2 = 23
ena = 18

enb = 19
in3 = 6
in4 = 5

temp1 = 1

GPIO.setmode(GPIO.BCM)
# -------motor 1 ----------
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
# -------motor 2 ----------
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
# --------pwm------------
p1 = GPIO.PWM(ena, 1000)
p2 = GPIO.PWM(enb, 1000)
p1.start(25)
p2.start(25)

# funciones de sentido de giro de los motores
def atras():
    p1.ChangeDutyCycle(35)
    p2.ChangeDutyCycle(35)
    GPIO.output(in1, GPIO.HIGH)  # motor 1 atras
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)  # motor 2 atras
    GPIO.output(in4, GPIO.LOW)


def adelante():
    GPIO.output(in1, GPIO.LOW)  # motor 1 adelante
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)  # motor 2 adelante
    GPIO.output(in4, GPIO.HIGH)


def GIRO_DERE():
    GPIO.output(in1, GPIO.LOW)  # MOTOR 1 adelante 
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)  # MOTOR 2 apagado 
    GPIO.output(in4, GPIO.LOW)

def GIRO_IZQ():
    GPIO.output(in1, GPIO.LOW)  # MOTOR 1 APAGADO 
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)  # MOTOR 2 ADELNATE
    GPIO.output(in4, GPIO.HIGH)

def GIRO_IZQ_FUERTE():
    GPIO.output(in1, GPIO.LOW)  # MOTOR 1 ATRAS
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)  # MOTOR 2 ADELANTE
    GPIO.output(in4, GPIO.LOW)





def GIRO_DERE_FUERTE():
    GPIO.output(in1, GPIO.LOW)  # MOTOR 1 ADELANTE
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)  # MOTOR 2 ATRAS
    GPIO.output(in4, GPIO.LOW)


def STOP():
    GPIO.output(in1, GPIO.LOW)  # MOTOR 1 APAGADO
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)  # MOTOR 2 APAGADO
    GPIO.output(in4, GPIO.LOW)
    
def BUSCAR():
    p1.ChangeDutyCycle(39)
    p2.ChangeDutyCycle(34)
    GIRO_DERE_FUERTE()
    

def encontrarPoligono (imagen):
    #Convertit la imagen de RGB a HSV
    hsv=cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
    #crearUmbrales
    bajos=np.array([45,61,138],dtype=np.uint8)
    altos=np.array([136,255,196],dtype=np.uint8)
    #Creando mascara
    mask=cv2.inRange(hsv,bajos,altos)

    kernel=np.ones((6,6), np.uint8)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

    aristas=cv2.Canny(mask,1,2)

    contornos,jerarquia=cv2.findContours(aristas,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    areas=[]
    for c in contornos:
        areas.append(cv2.contourArea(c))
    i=0
    p=0
    for areaActual in areas:
        if areaActual>150:
            figuraActual=contornos[i]
            lados=cv2.approxPolyDP(figuraActual,0.05*cv2.arcLength(figuraActual,True),True)
            numeroLados=len(lados)
            if numeroLados==3:
                cv2.drawContours(imagen,[figuraActual],0,(0,0,255),2)
                medidas=cv2.minAreaRect(figuraActual)
                if medidas[1][0]>p:
                    p=medidas[1][0]


        i=i+1
    return p

distancia=1
w=7
f=653
cap=cv2.VideoCapture(0)

while (1):
    ret, frame=cap.read()
    p = encontrarPoligono(frame)
    if p>0:
        distancia=(w*f)/p
    font=cv2.FONT_HERSHEY_SIMPLEX
    
    if distancia == 1 :
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        BUSCAR()
        print("buscando")
        
        
    if distancia >60 and distancia<80:
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        p1.ChangeDutyCycle(30)
        p2.ChangeDutyCycle(30)
        adelante()
        print("adelante")
     
    if distancia >80 and distancia<400:
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        BUSCAR()
        print("datos erroneos")
        print("buscar de nuevo ")
        
        
    
    elif distancia >40 and distancia<50:
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        p1.ChangeDutyCycle(25)
        p2.ChangeDutyCycle(25)
        adelante()
        print("adelante")
        
        
    elif distancia >30 and distancia<50:
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        p1.ChangeDutyCycle(20)
        p2.ChangeDutyCycle(20)
        adelante()
        print("adelante")
       
    elif distancia <30 and distancia > 25:
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        STOP()
        print("STOP")
        
    elif distancia < 25 and distancia > 5 : 
        mensaje="Distancia = "+str(round(distancia,2))+" cm"
        cv2.putText(frame,mensaje,(10,70),font,1,(0,0,255),2,cv2.LINE_AA)
        atras()
        print("reversa")
        
    cv2.imshow("Video",frame)
    
    
    k=cv2.waitKey(30) & 0xff
    if k ==27:
        STOP()
        break
    
cap.release()
cv2.destroyAllWindows()