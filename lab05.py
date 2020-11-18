import rospy
import tf
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#DECLARACOES DO ROSPY
od = Odometry()
scan = LaserScan()
rospy.init_node('no1')

#DECLARACOES DO PID
Kp = 1
Ki = 0.5
Kd = 0.5

#DECLARACOES DAS VARIAVEIS AUXILIARES
estado = 0
integrador = 0
erro_anterior = 0
tempo = 0
periodo = 0.0333333 #1/(2+0+1+7+0+0+3+6+3+8)

#LEITURA DO ANGULO
def lerAngulo(msg):
    quat = msg.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    euler = tf.transformations.euler_from_quaternion(q)
    angulo = euler[2]*180.0/math.pi
    return angulo

#LEITURA DA ODOMETRIA
def odomCallBack(msg):
    global od
    od = msg
    
#LEITURA DO SCAN
def scanCallBack(msg):
    global scan
    scan = msg


#FUNCAO DO TIMER
def timerCallBack(event):
    global Kp, Ki, Kd, integrador, erro_anterior, estado, tempo
    distancia = len(scan.ranges)
    leitura = min(scan.ranges[distancia-12:distancia+12])

    #Giro do robo
    if estado == 0:
        if leitura < 100:
            msg.angular.z = 0.3
        else:
            msg.angular.z = 0
            estado = 1    

    #Movimento do robo
    elif estado == 1:
        referencia = 0.5

        if leitura < 100:
            #Manobragem do erro
            erro = -(referencia - leitura)
            deltaerro = (erro - erro_anterior)/periodo
            erro_anterior = erro
            tempo += erro*periodo

            #PID    
            P = Kp * erro
            I = Ki * tempo
            D = Kd * deltaerro

        #---Controle    
        controle = P+I+D
        #Para evitar movimentos bruscos
        if controle > 1:
            controle = 1
        elif controle < -1:
            controle = -1
        msg.linear.x = controle
        #---Fim do controle

        #Fim do movimento
        if leitura >= 100:
            controle = 0
            estado = 2       
        
        msg = Twist() #Leitura
        msg.linear.x = controle #Mensagem de movimento linear
        pub.publish(msg) #Publica
    
        #Prevencao de erros no giro
        if leitura > 100:
            estado = 1
    
    #Estado final - conclusao do movimento
    else:
        msg.linear.x = 0
        msg.angular.z = 0
    
    #Prevencao de erro na leitura do scan
    if not(distancia > 0):
        msg.angular.z = 0
        msg.linear.x = 0

    #Publicacao do movimento pro robo
    pub.publish(msg)    

#FUNCOES DO ROSPY
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/od', Odometry, odomCallBack)
rospy.Subscriber('/scan', LaserScan, scanCallBack)
rospy.Timer(rospy.Duration(periodo), timerCallBack)
rospy.spin()