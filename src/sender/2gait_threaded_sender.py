#!/usr/bin/env python

import pika
import json
import threading
import time
import sys
import select
import os
import tty
import termios

from tkinter import *
import tkinter as tk
from  PIL import Image, ImageTk
import numpy as np
import math
#import _thread    #********************* I remommend that you use 'threading' imported above.
                   # in any case, pick one, NOT BOTH.  'threading' is more generally available and offors
                   # more capabilities.

from function_actions import *
from Experiment import Experiment
from Interaction import Interaction
from Anticipation import Anticipation

# GAIT
imgs = []

EXPERIENCES = {}
INTERACTIONS = {}
intendedInteraction = None
enactedInteraction = None
superInteraction = None
startState = False
isIntendedInteractionComposite = False

marginRectangle = 3
FRAME_WIDTH = 1200
FRAME_HEIGHT = 600
CANVAS_WIDTH = 1050
CANVAS_HEIGHT = 400
INVERVAL = 50
TITLE = 100

xValue = 0
yValue = CANVAS_HEIGHT-85

# messages corresponding to turtlebot's interactions
tif_message = {"id": "init-1", "plant-id": "init-1", "exchange": "test", "function-digit": "0"}
tr_message = {"id": "init-1", "plant-id": "init-1", "exchange": "test", "function-digit": "1"}
tl_message= {"id": "init-1", "plant-id": "init-1", "exchange": "test", "function-digit": "2"}
mf_message = {"id": "init-1", "plant-id": "init-1", "exchange": "test", "function-digit": "3" }
rr_message = {"id": "init-1", "plant-id": "init-1", "exchange": "test", "function-digit": "4"}
lr_message = {"id": "init-1", "plant-id": "init-1", "exchange": "test", "function-digit": "5"}
noa_message = {"id": "init-1", "plant-id": "init-1", "exchange": "test", "number-of-actions": True}

# feddback of gait's interactions
value = 0

# Create a lock for synchronization between threads
lock = threading.Lock()

# Create a table with links to images
interaction_images = {
    'e01':'images/move_forward_original.png',
    'e00':'images/bump_original.png',
    'e11':'images/turn_left.png',
    'e21':'images/turn_right.png',
    'e31':'images/feel_front_empty.png',
    'e30':'images/feel_front_wall.png',
    'e41':'images/feel_left_empty.png',
    'e40':'images/feel_left_wall.png',
    'e51':'images/feel_right_empty.png',
    'e50':'images/feel_right_wall.png'
}

# function to get user inputs
def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def addOrGetExperience(label,actionType):
    experiment = Experiment(label,actionType)
    EXPERIENCES[label] = experiment
    return EXPERIENCES[label]

def addOrGetPrimitiveInteraction(experiment,result,valence):
    label = experiment.label+str(result)
    if label not in INTERACTIONS:
        inter = Interaction(label)
        inter.experiment = experiment
        inter.result = result
        inter.valence = valence
        INTERACTIONS[label] = inter
    return INTERACTIONS[label]

def getDefaultAnticipations():
    anticipations = []
    for experimentLabel in EXPERIENCES:
        if EXPERIENCES[experimentLabel].isAbstract != True:
            anticipation = Anticipation(EXPERIENCES[experimentLabel],0)
            anticipations.append(anticipation)
    return anticipations

def getActivatedInteractions():
    contextInteractions = []
    if enactedInteraction is not None:
        contextInteractions.append(enactedInteraction)
        if not enactedInteraction.isPrimitive():
            contextInteractions.append(enactedInteraction.postInteraction)
        if superInteraction is not None:
            contextInteractions.append(superInteraction)
        
    activatedInteractions = []
    for interactionLabel in INTERACTIONS:
        if not INTERACTIONS[interactionLabel].isPrimitive():
            if INTERACTIONS[interactionLabel].preInteraction in contextInteractions:
                activatedInteractions.append(INTERACTIONS[interactionLabel])
    return activatedInteractions

def initial():
    e0 = addOrGetExperience("e0", 0);
    e0.resetAbstract()
    e1 = addOrGetExperience("e1", 1);
    e1.resetAbstract()
    e2 = addOrGetExperience("e2", 2);
    e2.resetAbstract()
    e3 = addOrGetExperience("e3", 3);
    e3.resetAbstract()
    e4 = addOrGetExperience("e4", 4);
    e4.resetAbstract()
    e5 = addOrGetExperience("e5", 5);
    e5.resetAbstract()

    v_moveSucess = 5
    v_moveFailture = -10
    v_turn = -3
    v_feelEmpty = -1
    v_feelWall = -2
    i01 = addOrGetPrimitiveInteraction(e0, 1, v_moveSucess);
    i00 = addOrGetPrimitiveInteraction(e0, 0, v_moveFailture);
    i11 = addOrGetPrimitiveInteraction(e1, 1, v_turn);
    i21 = addOrGetPrimitiveInteraction(e2, 1, v_turn);
    i31 = addOrGetPrimitiveInteraction(e3, 1, v_feelEmpty);
    i30 = addOrGetPrimitiveInteraction(e3, 0, v_feelWall);
    i41 = addOrGetPrimitiveInteraction(e4, 1, v_feelEmpty);
    i40 = addOrGetPrimitiveInteraction(e4, 0, v_feelWall);
    i51 = addOrGetPrimitiveInteraction(e5, 1, v_feelEmpty);
    i50 = addOrGetPrimitiveInteraction(e5, 0, v_feelWall);

    e0.intendedInteraction = i01;
    e1.intendedInteraction = i11;
    e2.intendedInteraction = i21;
    e3.intendedInteraction = i30;
    e4.intendedInteraction = i40;
    e5.intendedInteraction = i50;

def drawImage(canvas,label,xvalue,yvalue):
    print("label : ", label)
    source_image = Image.open(interaction_images[label])
    source_image = source_image.resize((50, 50), Image.ANTIALIAS)
    img = ImageTk.PhotoImage(source_image)
    x = canvas.create_image(xvalue,yvalue,anchor="nw", image=img)
    imgs.append(img)
    return img

def anticipate():
    anticipations = getDefaultAnticipations();
    #defaultAnticipations = getDefaultAnticipations();
    activatedInteractions =  getActivatedInteractions();

    anticipationIndex = False
    for activatedInteraction in activatedInteractions:
        #得到构建anticipation相应的experiment，proclivity，然后构建
        experimenttem = activatedInteraction.postInteraction.experiment
        proclivitytem = activatedInteraction.weight*activatedInteraction.postInteraction.valence
        anticipationtem = Anticipation(experimenttem,proclivitytem)

        for anticipation in anticipations:
            if anticipation.experiment == experimenttem:
                anticipation.addProclivity(proclivitytem)
                anticipationIndex = True
        if anticipationIndex == False:
            anticipations.append(anticipationtem)
        anticipationIndex = False
    return anticipations

def drawInteraction(canvas,intendedOrenactedInteraction,xvalue,yvalue):
    global marginRectangle,INVERVAL,isIntendedInteractionComposite
    intendedInteractionWidth = 0
    xvalueSource = xvalue
    yvalueSource = yvalue
    if intendedOrenactedInteraction.isPrimitive():
        image = drawImage(canvas, intendedOrenactedInteraction.label, xvalue, yvalue)
        intendedInteractionWidth = 75
        #canvas.create_rectangle(xvalueSource - marginRectangle,
        #                        yvalueSource - marginRectangle,
        #                        xvalueSource - marginRectangle + 50 + 2 * marginRectangle,
        #                        yvalueSource - marginRectangle + 50 + 2 * marginRectangle)
        if isIntendedInteractionComposite:
            canvas.create_rectangle(xvalueSource - marginRectangle,
                                    yvalueSource - marginRectangle,
                                    xvalueSource - marginRectangle + 50 + 2 * marginRectangle,
                                    yvalueSource - marginRectangle + 50 + 2 * marginRectangle)

    else:
        print('intendedOrenactedInteraction is not primitive')
        primitiveInteractionList = geInteractionList(intendedOrenactedInteraction)
        for primitiveInteraction in primitiveInteractionList:
            print('primitive interaction is:'+primitiveInteraction.strInteraction())
            image = drawImage(canvas, primitiveInteraction.label, xvalue, yvalue)
            xvalue += 75
        intendedInteractionWidth = len(primitiveInteractionList) * 75
        canvas.create_rectangle(xvalueSource-marginRectangle,
                                yvalueSource-marginRectangle,
                                xvalueSource-marginRectangle+intendedInteractionWidth-25+2*marginRectangle,
                                yvalueSource-marginRectangle+ INVERVAL+ 2*marginRectangle)
    return intendedInteractionWidth

# This function is the one for RMQ (I think)
def enact(intendedInteraction):
    global value
    result = None
    isWall = False
    if intendedInteraction.isPrimitive() == True:
        actionType = intendedInteraction.experiment.actionType
        if actionType == 0:         # move_forward
            producer_thread.channel.basic_publish(exchange='test', routing_key='init-1', body=json.dumps(mf_message))
            with lock:
                print("Published: ", mf_message)
                time.sleep(7.5)
            isWall = moveForward()
            result = value
            print("***result*** : ", result)
#             if isWall:
#                 result = 0
#             else:
#                 result = 1
        elif actionType == 1:       # left_rotate
            producer_thread.channel.basic_publish(exchange='test', routing_key='init-1', body=json.dumps(lr_message))
            with lock:
                print("Published: ", lr_message)
                time.sleep(5)
            isWall = turnLeft()
            result = 1
        elif actionType == 2:       # right_rotate
            producer_thread.channel.basic_publish(exchange='test', routing_key='init-1', body=json.dumps(rr_message))
            with lock:
                print("Published: ", rr_message)
                time.sleep(5)
            isWall = turnRight()
            result = 1
        elif actionType == 3:       # touch_in_front
            producer_thread.channel.basic_publish(exchange='test', routing_key='init-1', body=json.dumps(tif_message))
            with lock:
                print("Published: ", tif_message)
            isWall = feelFront()
            result = value
            print("***result*** : ", result)
#             if isWall:
#                 result = 0
#             else:
#                 result = 1
        elif actionType == 4:       # touch_left
            producer_thread.channel.basic_publish(exchange='test', routing_key='init-1', body=json.dumps(tl_message))
            with lock:
                print("Published: ", tl_message)
            isWall = feelLeft()
            print
            result = value
            print("***result*** : ", result)
#             if isWall:
#                result = 0
#             else:
#                result = 1
        elif actionType == 5:       # touch_right
            producer_thread.channel.basic_publish(exchange='test', routing_key='init-1', body=json.dumps(tr_message))
            with lock:
                print("Published: ", tr_message)
            isWall = feelRight()
            result = value
            print("***result*** : ", result)
#             if isWall:
#                 result = 0
#             else:
#                 result = 1

        inter = addOrGetPrimitiveInteraction(intendedInteraction.experiment,result,6)
        return inter

    else:
        enactPreInteraction = enact(intendedInteraction.preInteraction)
        if enactPreInteraction != intendedInteraction.preInteraction:
            return enactPreInteraction
        else:
            enactedPostInteraction = enact(intendedInteraction.postInteraction)
            return addOrGetAndReinforceCompositeInteraction(enactPreInteraction, enactedPostInteraction,False)

def learnCompositeInteraction(currentEnactedInteraction):
    print
    print("function learnCompositeInteraction")
    global enactedInteraction,superInteraction
    if enactedInteraction != None:
        print('previous enactedInteraction is:'+enactedInteraction.strInteraction())
    print('current  enactedInteraction is:'+currentEnactedInteraction.strInteraction())
    if superInteraction != None:
        print('superInteraction is:'+superInteraction.strInteraction() )
    previousEnactedInteraction = enactedInteraction
    lastEnactedInteraction = currentEnactedInteraction
    previousSuperInteraction = superInteraction
    lastSuperInteraction = None
    if previousEnactedInteraction != None:
        lastSuperInteraction = addOrGetAndReinforceCompositeInteraction(previousEnactedInteraction, lastEnactedInteraction,True)
    if previousSuperInteraction != None:
        addOrGetAndReinforceCompositeInteraction(previousSuperInteraction.preInteraction, lastSuperInteraction,True)
        addOrGetAndReinforceCompositeInteraction(previousSuperInteraction, lastEnactedInteraction,True)
    superInteraction = lastSuperInteraction

def addOrGetAndReinforceCompositeInteraction(previousInteraction,lastInteraction,isFlag):
    label = "<"+previousInteraction.label+lastInteraction.label + ">"
    if label in INTERACTIONS:
        if isFlag:
            print("enforce interaction:"+INTERACTIONS[label].strInteraction())
            INTERACTIONS[label].incrementWeight()
    else:
        inter = Interaction(label)
        inter.preInteraction = previousInteraction
        inter.postInteraction = lastInteraction
        inter.incrementWeight()
        inter.valence = previousInteraction.valence+lastInteraction.valence
        inter.experiment = addOrGetAbstractExperiment(inter)
        INTERACTIONS[label] = inter
        print("learn new interaction:"+INTERACTIONS[label].strInteraction() )
    return INTERACTIONS[label]

def addOrGetAbstractExperiment(interaction):
    label = interaction.label.replace('e', 'E').replace('>', '|')
    if label not in EXPERIENCES:
        abstractExperiment = Experiment(label,6)
        abstractExperiment.intendedInteraction = interaction
        EXPERIENCES[label] = abstractExperiment
    return EXPERIENCES[label]

def geInteractionList(interaction):
    interactionList = []
    tempList = []
    tempList.append(interaction)
    while len(tempList):
        tempInteraction = tempList.pop()
        if not tempInteraction.isPrimitive():
            tempList.append(tempInteraction.postInteraction)
            tempList.append(tempInteraction.preInteraction)
        else:
            interactionList.append(tempInteraction)
    return interactionList

def start(canvas):
    global startState
    print('start the process')
    if not startState:
        startState = True
        # **************************
        #  don't use _thread!!! # _thread.start_new_thread(myProducerThread.run, (myProducerThread, canvas, 1))
        # Lets do it the 'threading way'

        # Create new threads
        global producer_thread
        global consumer_thread
        producer_thread = myProducerThread(canvas, 1)
        consumer_thread = myConsumerThread()
        consumer_thread.daemon = True

        # Start new Threads
        consumer_thread.start()
        producer_thread.start()

        # Debugging
        print("canvas : ", canvas)
        print("count : ", 1)

def stop():
    print('stop the process')
    global startState
    # myConsumerThread.run()

def close(root):
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate

def bytes_to_string (bytes):
    return "".join( chr(x) for x in bytes)

def consumer_callback(ch, method, properties, body):
    # Get lock to synchronize threads
    global value
    with lock:
      print(" [x] Received %r" % body)
      str_body = bytes_to_string(body)
      # print("str_body : ", str_body)
      observations = json.loads(str_body)                               # .decode("utf-8")
      if "observations" in observations:
        # print("observations : ", observations["observations"][0])
        value = observations["observations"][0]["value"][0]             # json.dumps(json_data["data"])
        print("value : ", value)
      # traiter le message ici

class myConsumerThread (threading.Thread):
   def __init__(self):
      threading.Thread.__init__(self)
   def run(self):
      print("Starting myConsumerThread")
      self.connection = pika.BlockingConnection(pika.ConnectionParameters('localhost', 5672))
      self.channel = self.connection.channel()
      self.channel.queue_declare(queue='test')
      self.channel.exchange_declare(exchange='test', exchange_type='topic')
      self.result = self.channel.queue_declare(queue='observations', exclusive=False)
      self.queue_name = self.result.method.queue
      self.channel.queue_bind(exchange='test', queue=self.queue_name)
      self.channel.basic_consume(consumer_callback, queue='observations', no_ack=True)

      with lock:
        print(' [*] Waiting for messages. To exit press CTRL+C')

      self.channel.start_consuming()

class myProducerThread (threading.Thread):
   def __init__(self, canvas, count):
      threading.Thread.__init__(self)
      self.canvas = canvas
      self.count = count
   def run(self):
      print("Starting myProducerThread")
      self.connection = pika.BlockingConnection(pika.ConnectionParameters('localhost', 5672))
      self.channel = self.connection.channel()
      self.channel.exchange_declare(exchange='test', exchange_type='topic')

      global startState, xValue, yValue, enactedInteraction, CANVAS_HEIGHT, INVERVAL, isIntendedInteractionComposite
      intendedInteractionWidth = 0
      if startState:
          loopNum = 1
          while loopNum < 200:
                print('------------------------------------')
                print('loopNum is:'+str(loopNum))
                if loopNum == 1:
                    initial()
                anticipations = anticipate()
                sortedAnticipations = sorted(anticipations, key=lambda anticipation: anticipation.proclivity,reverse=True)
                intendedInteraction = sortedAnticipations[0].experiment.intendedInteraction

                print('intendedInteraction is:' + intendedInteraction.strInteraction())
                if not intendedInteraction.isPrimitive():
                    isIntendedInteractionComposite = True
                intendedInteractionWidth = drawInteraction(self.canvas,intendedInteraction,xValue,yValue)

                actionType = intendedInteraction.experiment.actionType
                pretentResult = intendedInteraction.result
                currentEnactedInteraction = enact(intendedInteraction)

                print('enactedInteraction is:' + currentEnactedInteraction.strInteraction())
                if enactedInteraction == None:
                    print('previous enactedInteraction is:None')
                else:
                    print('previous enactedInteraction is:' + enactedInteraction.strInteraction())
                learnCompositeInteraction(currentEnactedInteraction)

                enactedInteraction = currentEnactedInteraction
                drawInteraction(self.canvas, enactedInteraction, xValue+12, yValue+12)
                if isIntendedInteractionComposite:
                    isIntendedInteractionComposite = False

                #drawIntendedEnactedInteraction(canvas, intendedInteraction, enactedInteraction)
                self.canvas.create_text(xValue+(intendedInteractionWidth-25)/2, CANVAS_HEIGHT-INVERVAL/2+7, anchor="nw", text=str(loopNum))

                loopNum += 1
                xValue+=intendedInteractionWidth
                if xValue > 1050:
                    self.canvas["scrollregion"] = "%d %d %d %d" % (0, 0, xValue, 500)
                    self.canvas.xview_moveto(1)
                print
                time.sleep(0.2)


# GAIT
window = tk.Tk()
frame = tk.Frame(window,width=FRAME_WIDTH,height=FRAME_HEIGHT)
frame.pack(expand=True,fill=BOTH)


window_height = 600
window_width = 1200
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()
x_cordinate = int((screen_width/2) - (window_width/2))
y_cordinate = int((screen_height/2) - (window_height/2))
window.geometry("{}x{}+{}+{}".format(window_width, window_height, x_cordinate, y_cordinate))
window.title = "GAIT"

canvas = tk.Canvas(frame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg='white',scrollregion=(0,0,CANVAS_WIDTH,CANVAS_HEIGHT))
canvas.place(x=150,y=TITLE)
hbar=Scrollbar(frame,orient=HORIZONTAL)
hbar.pack(side=BOTTOM,fill=X)
hbar.config(command=canvas.xview)
canvas.config(xscrollcommand=hbar.set)
tk.Label(frame,text="Generating and Analyzing Interaction Traces Toolkit (GAIT)", font=("Helvetica",16)).place(x=350,y=30)
tk.Label(frame,text="loop number:").place(x=50,y=TITLE+CANVAS_HEIGHT-INVERVAL/2)
tk.Label(frame,text="Enacted Interaction:").place(x=20,y=TITLE+CANVAS_HEIGHT-INVERVAL)
tk.Label(frame,text="Intended Interaction:").place(x=20,y=TITLE+CANVAS_HEIGHT-(INVERVAL/2)*3)
tk.Label(frame,text="Composite Interaction:").place(x=15,y=TITLE+CANVAS_HEIGHT-(INVERVAL/2)*7)

startButton = tk.Button(frame, text="Start", width=10,height=1, command=lambda:start(canvas)).place(x=500,y=TITLE+CANVAS_HEIGHT+INVERVAL/2)
stopButton = tk.Button(frame, text="Stop", width=10,height=1, command=stop).place(x=600,y=TITLE+CANVAS_HEIGHT+INVERVAL/2)
closeButton = tk.Button(frame, text="Close", width=10,height=1, command=close).place(x=700,y=TITLE+CANVAS_HEIGHT+INVERVAL/2)

window.mainloop()

# ********************** all of this section probably belongs in 'start' and 'stop' and not here at the top level!
# Create new threads
# consumer_thread = myConsumerThread()
# consumer_thread.daemon = True  # when the main thread ends, so will this thread.

# Wrong place to put this!
# producer_thread = myProducerThread()

# Start new Threads
# consumer_thread.start()
# Wrong place for this, you need to start it after having initialized it, see start function above
# producer_thread.start()

# Wait for the producer thread to complete
producer_thread.join()
time.sleep(5)   # Give the consumer 5 seconds to consume the last messages

print("Exiting Main Thread")
