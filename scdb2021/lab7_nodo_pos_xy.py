#!/usr/bin/python
import rospy
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as efq

class NodePosition():
    def __init__(self):
        #Esta funcion nos permite inicializar los diferentes elementos del nodo.
        #Vamos a utilizar la libreria rospy que permite interactuar con ROS usando python.
        #Para eso creamos una instancia de esta libreria en la clase Node()
        self.rospy = rospy
        #Inicializamos el nodo con el nombre que aparece en el primer argumento.
        self.rospy.init_node("node_position", anonymous = True)
        #Inicializamos los parametros del nodo
        self.initParameters()
        #Creamos los suscriptores del nodo
        self.initSubscribers()
        #Creamos los publicacdores del nodo
        self.initPublishers()
        #Vamos a la funcion principal del nodo, esta funcion se ejecutara en un loop.
        self.main()
        return

    def initParameters(self):
        #Aqui inicializaremos todas las variables del nodo
        self.kp = 1
        self.ki = 1
        self.kd = 1
	self.kn = 0
        self.xe_prev = 0
        self.v_limit = 0.5
        self.topic_odom = "/odometry/filtered"
        self.topic_vel = "/cmd_vel"
        self.topic_error = "/error"
        self.change_odom = False
        self.change_set = False
	self.yy=0
	#self.y =np.linspace(-2*math.pi,2*math.pi,len(self.t)+1)
	#self.xrd =[2*math.cos(self.yy) + 2 for self.yy in self.y]
	#self.yrd = [2*math.sin(self.yy)  for self.yy in self.y]
	self.t = np.linspace(0,20,200)
	self.y =np.linspace(-4,4,len(self.t)+1)
	self.xrd =[4*math.cos(self.yy)-(math.sin(self.yy)) for self.yy in self.y];
	self.yrd = [2*math.sin(self.yy)*math.cos(self.yy) for self.yy in self.y]; 	
	self.rate = self.rospy.Rate(30)       
	return

    def callback_odom(self, msg):
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.phi = efq([quat.x, quat.y, quat.z, quat.w])[2]
        self.change_odom = True
        return

    def initSubscribers(self):
        #Aqui inicializaremos los suscriptrores
        self.sub_odom = self.rospy.Subscriber(self.topic_odom, Odometry, self.callback_odom)
        return

    def initPublishers(self):
        #Aqui inicializaremos los publicadores
        self.pub_vel = self.rospy.Publisher(self.topic_vel, Twist, queue_size = 10)
        self.pub_error = self.rospy.Publisher(self.topic_error, Point, queue_size = 10)
        return

    def controller(self):
        self.xe=self.xrd[self.kn+1] - self.xr
        self.ye= self.yrd[self.kn+1] - self.yr
	if (abs(self.xe)<1) and (abs(self.ye)<1):
	    self.kn=self.kn+1
        self.e = [[self.xe],
                  [self.ye]]
        self.K = [[1.5, 0],
                  [0, 1.5]]
	self.h = [[self.xrd[self.kn+1] - self.xrd[self.kn]],[self.yrd[self.kn+1] - self.yrd[self.kn]]]
        self.jacob = [[np.cos(self.phi), -0.5*np.sin(self.phi)],
                      [np.sin(self.phi),  0.5*np.cos(self.phi)]]
        self.vc = np.matmul(np.linalg.inv(self.jacob), np.matmul(self.K, self.e)+self.h)
        self.v = 0.7*np.tanh(0.5*self.vc[0][0])
        self.w = 0.1*np.tanh(0.5*self.vc[1][0])
        return

    def makeVelMsg(self):
        self.msg_vel = Twist()
        self.msg_vel.linear.x = self.v
        self.msg_vel.angular.z = self.w
        return

    def makeErrorMsg(self):
        self.msg_error = Point()
        self.msg_error.x = self.xe
        self.msg_error.y = self.ye
        return

    def main(self):
        #Aqui desarrollaremos el codigo principal
        print("Nodo OK")
	#self.kn=0
        while not self.rospy.is_shutdown():
            if self.change_odom:
                self.controller()
                self.makeVelMsg()
                self.makeErrorMsg()
                self.pub_vel.publish(self.msg_vel)
                self.pub_error.publish(self.msg_error)
                self.change_odom= False
		#self.kn=self.kn+1
            self.rate.sleep()

if __name__=="__main__":
    try:
        print("Iniciando Nodo")
        obj = NodePosition()
    except rospy.ROSInterruptException:
        print("Finalizando Nodo")
