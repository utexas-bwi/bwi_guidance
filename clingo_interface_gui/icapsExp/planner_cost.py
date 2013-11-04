#! /usr/bin/env python

import os
import signal
import subprocess
import sys
import time
import threading

#import roslib; roslib.load_manifest('clingo_interface_gui')
import rospy

# Brings in the SimpleActionClient
import actionlib

import clingo_interface_gui.msg



domainFile = sys.argv[1]
queryFile = sys.argv[3]
initialFile = sys.argv[2]

action = ["sense","goto","gothrough","callforopen","approach"]
fluent = ["cost", "at", "open", "visited","beside", "n_open", "n_visited", "n_beside", "inside","n_inside","goal","facing","n_facing","believeinside","n_believeinside","served","n_served"]
newknowledge = ["inside"]
believeinside = []
passto = []

def UpdateEnvironmentalKnowledge(fluentarray,arg1,arg2):
	header = fluentarray[0]
	index = header.index(arg2)
	print "Update environment knowledge for "+head[0][0]+"\n"
	for i in range(1,len(fluentarray)):
		if fluentarray[i][0] == arg1:
			for j in range(1,len(fluentarray[i])):
				if j!=index:
					fluentarray[i][j]="false"
				else:
					fluentarray[i][j]="true"
			break

def OutputEnvironmentalKnowledge(fluentarray,fname):
	oenvfile = open(fname,"w")
	for e in fluentarray:
		for i in range(len(e)-1):
			oenvfile.write(e[i]+",")
		oenvfile.write(e[len(e)-1]+"\n")
	oenvfile.close()

def GenerateEnvironmentalKnowledge(s):
	print "Generate environment knowledege for "+s+"\n"
	array=[]
	envFile = open(s,"r")
	initialenv = open("initialenv","a")
	flag = 0
	for line in envFile:
		if flag == 0:
			header = line.rstrip().split(",")
			array.append(header)
			flag = 1
		else:
			linevalue = line.rstrip().split(",")
			array.append(linevalue)
			for i in range(len(linevalue)-1):
				if s=="believeinside":
					initialenv.write("h(eql("+s+"("+linevalue[0]+","+ header[i+1].rstrip()+"),"+ linevalue[i+1].rstrip()+"),0).\n")
				else:
					initialenv.write("h(eql("+s+"("+linevalue[0]+","+ header[i+1].rstrip()+"),"+ linevalue[i+1].rstrip()+")).\n")
	
	envFile.close()	
	initialenv.close()
	
        return array
			
class Command(object):
    def __init__(self, cmd):
        self.cmd = cmd
        self.process = None
        self.outfile = open("result","w")

    def run(self, timeout):
        def target():
            print 'Thread started'
            self.process = subprocess.Popen(self.cmd, shell=True, stdout=self.outfile, preexec_fn=os.setsid)
            self.process.communicate()
            print 'Thread finished'

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            print 'Terminating process'
            os.killpg(self.process.pid, signal.SIGTERM)
            thread.join()
        print "Process RetCode: " + str(self.process.returncode)
        self.outfile.close()
        return self.process.returncode

def GeneratePlan():
        plan = []
        states = []
	ienv = open("initialenv","w")
	ienv.close()

	believeinside = GenerateEnvironmentalKnowledge("believeinside")
	passto = GenerateEnvironmentalKnowledge("passto")
        clingo_command = Command("clingo -c maxstep=40 --opt-heu distances.lua initialenv show.asp "+domainFile+" "+initialFile+" "+queryFile)
        clingo_command.run(60)

        inputFile = open("result","r")
        if inputFile.readline()=="UNSATISFIABLE\n":
                print "goal not achieved at 18 steps incremented..."
        else:
                linelist = []
                for line in inputFile:
                        linelist.append(line)
                        if line[:11] == "SATISFIABLE":
				optimization_line = linelist[-2]
				plan_line = linelist[-3]
                print optimization_line
		words = plan_line.split()
		for w in words:
			if w.find("(")!=-1:
				a1 = w[:w.find("(")]
				a2 = w[w.find("(")+1:w.rfind(")")]
				a3 = a2[:a2.rfind(",")]
				t1 = a2[a2.rfind(",")+1:]
				if w[:w.find("(")] in action:
					plan.append((int(t1),[a1]+a3.split(",")))
				if w[:w.find("(")] in fluent:
					states.append((int(t1),[a1]+a3.split(",")))
		plan = sorted(plan, key=lambda tup: tup[0])
		states = sorted(states, key=lambda tup: tup[0])

        inputFile.close()
        print plan
#        print states
        return (plan,states)

def PlannerClient():
	print "Planner started"
	client = actionlib.SimpleActionClient('clingo_interface_gui', clingo_interface_gui.msg.ClingoInterfaceAction)
    	client.wait_for_server()

#initial state sensing:

	command = clingo_interface_gui.msg.ClingoFluent("noop",[])
	sensedfluent =  clingo_interface_gui.msg.ClingoFluent()
	evalfluent = []
	
	goal = clingo_interface_gui.msg.ClingoInterfaceGoal(command, sensedfluent, evalfluent)
	client.send_goal(goal)
	client.wait_for_result()
	result = client.get_result()

	
	inputFile = open(sys.argv[2],"w")
	for fluent in result.observable_fluents:
		op = fluent.op
		arg = fluent.args
		curstate = (str(0),[op]+arg)
		s="("
		for i in range(1,len(curstate[1])):
			s = s + curstate[1][i]+","
			s = s+"0).\n"
			newinit = curstate[1][0]+s
		inputFile.write(newinit)
		print "Initial sensing:"+ newinit
	inputFile.close()

	result = GeneratePlan()
	SendoutPlan(result,client)

#task finished, send out "finish" action
	finish = clingo_interface_gui.msg.ClingoFluent("finish",[])
	fsensedfluent =  clingo_interface_gui.msg.ClingoFluent()
	fevalfluent = []
	
	goalfinished = clingo_interface_gui.msg.ClingoInterfaceGoal(finish,fsensedfluent, fevalfluent)
	client.send_goal(goalfinished)
	print "Finish goal"
	client.wait_for_result()
	

def SendoutPlan(result,client):
	plan = result[0]
	state = result[1]

	command = clingo_interface_gui.msg.ClingoFluent()
	sense_fluent = clingo_interface_gui.msg.ClingoFluent()
#	evaluate_fluent = clingo_interface_gui.msg.ClingoFluent()
	evaluate_fluent =[]
	for i in range(len(plan)):
    		s = plan[i]
		if s[1][0] == "sense":
			command.op = "sense"
			w = s[1][1] # the fluent to be sensed
			sense_fluent.op = w[:w.find("(")]
			if w.find("(")!=-1:
				a2 = w[w.find("(")+1:w.rfind(")")] # list of arguments of sense
				sense_fluent.args = a2.split(",")
		else:
			command = clingo_interface_gui.msg.ClingoFluent(s[1][0], [s[1][1]])
		

		print "action:", s[1][0], s[1][1]
 		goal = clingo_interface_gui.msg.ClingoInterfaceGoal(command, sense_fluent, evaluate_fluent)
#		print goal
		client.send_goal(goal)
		client.wait_for_result()
		result= client.get_result()
#		print result
		expectstate = []
		for ss in state:
			if (i+1 < len(plan)):
				nexttime = plan[i+1][0]
			else:	
				nexttime = s[0]+1		
			if ss[0]==nexttime:
				expectstate.append(ss)
	#		print expectstate

		needreplan = 0
		for fluent in result.observable_fluents:
			op = fluent.op
			if op == "visited":
				 continue
			arg = fluent.args
			if (i+1 < len(plan)):
				nexttime = plan[i+1][0]
			else:	
				nexttime = s[0]+1		

			curstate = (nexttime,[op]+arg)
				
			if curstate in expectstate:	
				print "observation as expected: ", curstate
			else:
				print "unexpected observation: ", curstate
				needreplan = 1
				break
	
		if needreplan == 0:
			for se in expectstate:
				if se[1][0]=="goal":
					goalinfo = "goal("+ se[1][1] + ","+str(se[0])+")."
					print "archieve "+goalinfo		
					inputFile3 = open(sys.argv[3],"a")
					inputFile3.write(goalinfo)
					inputFile3.close()
		if needreplan == 1:
			#generate a new initial state and calling for replan
			inputFile = open(sys.argv[2],"w")
			for fluent in result.observable_fluents:
				op = fluent.op
				arg = fluent.args				
				curstate = (plan[i+1][0]-1,[op]+arg)
			#	print curstate
				s="("
				for p in range(1,len(curstate[1])): #arguments from second elem in list
					s = s + curstate[1][p]+","
				s = s+"0)."
				newinit = curstate[1][0]+s

				
				print "New initial:" + newinit
				if op in newknowledge:
					if op=="inside":
						person = arg[0]
						room = arg[1]
						UpdateEnvironmentalKnowledge(believeinside,person,room)
						print believeinside
						OutputEnvironmentalKnowledge(believeinside,"believeinside")				

				else:
					inputFile.write(newinit)
			inputFile.close()
			newresult = GeneratePlan()
			
			SendoutPlan(newresult,client)
			break



if __name__ == '__main__':
#	print GeneratePlan()
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('asp_planner_client_py')
        PlannerClient()
  #      print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


