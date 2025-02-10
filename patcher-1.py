# Patcher, ver 0.1
# python script for scheduler patch synthesis, under unicore NPEDF/NPRM

import logging
from z3 import *
from operator import itemgetter
from itertools import combinations

def printtrace(tr,trmsg=''): 
	#logging.info(trmsg+' (t,j,r,s,e,d,chain,hit,drop): \n'+'\n'.join(str(i) for i in tr))
	pass

def printtraj(tr):
	xval = []
	for cc in range(len(A)):
		for dd in range(len(A[cc])):
			print('xval',cc,dd,':::')
			print([tr[x[cc][dd][i]].numerator().as_long()/tr[x[0][0][i]].denominator().as_long() for i in  range(len(x[cc][dd]))])

def gethitsequence(tr):
	tmp1 = []
	for cid in range(len(A)):
		for dimm in range(len(A[cid])):
			tmp1 = [tr[x[cid][dimm][i]].numerator().as_long()/tr[x[cid][dimm][i]].denominator().as_long() for i in range(len(x[cid][dimm]))]
			#logging.info('control sys '+str(cid)+', dim '+str(dimm)+', trace: '+str(tmp1))
			if dimm==len(A[cid])-1: continue # do not check for violations for last dim i.e. u 
			# ~ if not dimm==0: continue # do not check for violations other than 0th dim
			violations = []
			for jid in range(len(tmp1)):
				if jid==(lasso2//period[cid]):
					violations.append((tmp1[(lasso2//period[cid])] < initstate[cid][dimm][0]) or (tmp1[(lasso2//period[cid])] > initstate[cid][dimm][1]))
					# ~ if tmp1[(lasso1//period[cid])]>0: violations.append(tmp1[(lasso1//period[cid])] < tmp1[(lasso2//period[cid])])
					# ~ else: violations.append(tmp1[(lasso2//period[cid])] < tmp1[(lasso1//period[cid])])
				elif dimm==0:
					violations.append((tmp1[jid] > upperbound[cid][jid]) or (tmp1[jid] < lowerbound[cid][jid]))
				else:
					violations.append(False)
			if True in violations:
				logging.info('safety violated: '+str(violations))
				taskrun = getsortedtrace(tr)
				hm = [(hit1,drop1) for (t1,j1,r1,s1,e1,d1,chain1,hit1,drop1) in taskrun if t1==cid] # extract hit/miss seq for the controller 
				return (cid,hm,violations)
	assert False, 'unexpected, no violation found for any trace!'		

def getsortedtrace(tr):
	trace = []
	for t in range(ntasks):
		for j in range(jobs[t]):
			rr = tr[r[t][j]].as_long() # extract release time
			ss = tr[s[t][j]].as_long() # extract start time
			ee = tr[e[t][j]].as_long()
			cc = is_true(tr[ch[t][j]])
			hh = is_true(tr[hit[t][j]])
			dr = is_true(tr[d[t][j]])
			if tasktype[t] == 'P': dd = (j+1)*period[t]+offset[t] # compute deadline
			else: dd = rr+period[t]
			trace += [(t,j,rr,ss,ee,dd,cc,hh,dr)] # (t,j,r,s,e,d,chain,dmiss,drop)
	sortedst = sorted(trace, key=itemgetter(3,4)) # sort on key (start,tid)
	return sortedst 

def staticprecedence(job1,job2): # check if first precedes second statically
	t1,j1 = job1
	t2,j2 = job2
	dl1, dl2 = period[t1]*(j1+1)+offset[t1], period[t2]*(j2+1)+offset[t2] # works for both P->P, P->S 
	j1releasedearlier = (j1*period[t1]+offset[t1]+jitter[t1] <= j2*period[t2]+offset[t2])
	if t1==t2:
		if j1==j2: assert False, 'unexpected'
		if j1<j2: return True # intra-task precedence is assumed always
		else: return False
	if tasktype[t1]=='S': return False # S->S, S->P both is false
	if schedule == 'np-edf': # job1 is periodic
		if (dl1 < dl2) and j1releasedearlier: return True
		else: pass
	elif (period[t1]<period[t2]) and j1releasedearlier: return True # np-rm
	return False

def systemspec():
	global offset,period,bcet,wcet,jitter,tasktype,tasksetname,horizon,schedule,A,B,K,R,initstate,strategy, ntasks,nperiodic,nsporadic,jobs,idealtrace,maxdev
	global lasso1,lasso2
	global upperbound, lowerbound
	offset=[0,0,0,0] # always set offsets to 0
	
	## edit the following lines for setting up the task specification
	period=[200,100,100,200] # set up the task periods
	bcet,wcet=[50,30,20,10],[100,55,50,30] # bcet, wcet values 
	jitter=[2,1,1,0] # small percentage of period
	tasktype=['P','P','P','S'] # either 'P' for periodic, or 'S' for sporadic
	tasksetname = 'F1-RC-DC' # give a name to the task set
	schedule = 'np-edf' # 'np-rm' # use only NP-EDF scheduling policy, for now
	lasso1 = 0  # DO NOT change this
	lasso2 = 4*200 # k*LCM # the analysis horizon, a multiple of LCM(periods). Higher value here will significantly impact analysis time as number of jobs spawned will be higher. 
	horizon = lasso2 # DO NOT change this
	## task specification setup ends
	
	## set up the control specifications
	# F1Tenth @200ms
	A0 = [[1.0, 1.3, 5.11811024], [0.0, 1.0, 3.93700787], [0.0, 0.0, 0.0]] # A matrix is a list of lists
	B0 = [0.0, 0.0, 1.0] # B matrix is a list
	K0 = [0.12847214, 0.41262919, 1.62452438] # control gains matrix is a list
	R0 = [0.0] # preferably set the R values to 0 for all control systems
	# ~ initstate0 = [[0.5, 1.0],[0.5, 1.0],[0.0,0.0]] # [min,max] range for each dimension 
	initstate0 = [[-1.0, 1.0],[-2.0, 1.0],[0.0,0.0]] # [min,max] range for each dimension 
	# ~ initstate0 = [[-1.0, 1.0], [-2.0, 1.1], [0.0, 0.0]] # [min,max] values for each dimension. NOTE: last dimension min,max should be set to 0 as it is the augmented dimension
	maxdev0 = 4.0 # maximum deviation from reference, for all dimensions 
	c0x0 = [1, 2.3, 0.8305837346450505, 0.23594052355478845, 0.061172194211656004, 0.015180227978747886, 0.0036805002650110996, 0.0008808360352138991, 0.00020923847402964628, 4.948738822135045e-05, 1.167430773579879e-05, 2.749830260052804e-06, 6.471232026822143e-07, 1.522065893295259e-07, 3.57882090211035e-08, 8.413232807668398e-09, 1.9775885236918084e-09, 4.648139337338285e-10, 1.0924574354352305e-10, 2.5675520900137504e-11, 6.034310289186524e-12, 1.4181827880558376e-12, 3.3329938279855713e-13, 7.833132094552705e-14, 1.8409228125137012e-14, 4.326485210172843e-15, 1.0167977521369781e-15, 2.3896470367468088e-16, 5.616074176403078e-17, 1.3198721055628498e-17, 3.101921683954092e-18] # this is the ideal or nominal trajectory (obtained without any deadline misses), ONLY FOR 0th DIMENSION
	c0x0 = [0]*len(c0x0) # enable this for safety BOX instead of safety PIPE

	# RC network@100ms
	A1 = [[0.54947186, 0.07239801, 0.28308422], [0.0144796, 0.93318132, 0.0069334], [0.0, 0.0, 0.0]]
	B1 = [0.0, 0.0, 1.0]
	K1 = [0.05886312, 0.07868521, 0.03028579]
	initstate1 = [[0.0, 1.0],[0.0, 1.0],[0.0,0.0]] # [min,max] range for each dimension 
	# ~ maxdev1 = 2.0 #
	maxdev1 = 1.5 #
	c1x0 = [1, 0.6218698685461647, 0.371370995861039, 0.23837338218376325, 0.166456954120747, 0.12638059024177287, 0.10298354645028139, 0.08839996544765089, 0.07854374545227186, 0.07128755077762707, 0.06551943903683383, 0.06065412168722863, 0.05637974045196319, 0.05252665040595633, 0.04899940857367611, 0.0457415072138286, 0.042717072064984575, 0.039901351059363065, 0.03727575874679051, 0.03482528257363583, 0.03253711441524954, 0.030399918455690376, 0.028403430237614394, 0.02653822869189336, 0.024795599106345746, 0.02316744444685374, 0.021646222888102203, 0.020224900007084788, 0.018896909586510163, 0.01765611982596122, 0.016496803239786077] # ideal trajectory for 0th dim

	# DC motor @100ms
	A2 = [[0.36783052, 0.05635456, 0.01054672], [-0.00112709, 0.81866696, 0.16427936], [ 0.0, 0.0, 0.0]]
	B2 = [0.0, 0.0, 1.0]
	K2 = [0.00080453, 0.15106493, 0.03031163]
	# ~ initstate2 = [[0.0, 1.0],[0.0, 1.0],[0.0,0.0]] # [min,max] range for each dimension 
	initstate2 = [[-0.1, 1.0],[-0.1, 1.0],[0.0,0.0]] # [min,max] range for each dimension 
	maxdev2 = 1.0 #
	# ~ maxdev2 = 0.6 #
	c2x0 = [1, 0.42418507604914224, 0.20049858884069355, 0.10877668442515992, 0.06761047128217773, 0.04661906317507053, 0.034289554738047116, 0.02612301168962445, 0.020257204137833645, 0.015843986028476423, 0.012442891743489266, 0.009790670061073137, 0.0077107128410593756, 0.006075185549651465, 0.004787513758840264, 0.003773118510553083, 0.00297378455190727, 0.0023438362212448377, 0.0018473495824671666, 0.001456038337079545, 0.0011476181796241406, 0.0009045288900827828, 0.0007129312618675035, 0.0005619180274784946, 0.0004428924878364241, 0.000349078967164459, 0.0002751370394380879, 0.00021685749741059482, 0.00017092273179925103, 0.00013471787075577965, 0.00010618192517845292]
	
	# final control spec, concatenate the respective matrices
	A = [A0,A1,A2]
	B = [B0,B1,B2]
	K = [K0,K1,K2]
	R = [[0.0],[0.0],[0.0]]
	initstate=[initstate0,initstate1,initstate2]
	idealtrace = [c0x0,c1x0,c2x0] # ideal trace always wrt dimension 0! 
	maxdev = [maxdev0,maxdev1,maxdev2] 
	
	upperbound, lowerbound = [],[] # safety bounds
	for cid in range(len(A)): # compute safety bounds wrt ideal trace
		upperbound.append([ii+maxdev[cid] for ii in idealtrace[cid]])
		lowerbound.append([ii-maxdev[cid] for ii in idealtrace[cid]])
				
	strategy = 'zero-and-kill' # use this for now
	# ~ strategy = 'hold-and-kill' 
	
	# basic sanity checking of specs
	assert len(A)==len(B)==len(K)==len(R)==len(initstate), 'bug'
	assert all(o==0 for o in offset), 'offsets should be zero'
	# ~ assert all(tt=='P' for tt in tasktype), 'only periodic tasks!'
	assert len(offset)==len(period)==len(bcet)==len(wcet)==len(jitter)==len(tasktype), 'task spec bug!'
	assert all(o>=0 for o in period+bcet+wcet+jitter), 'negative value found'
	assert all(bcet[i]<=wcet[i] for i in range(len(bcet))), 'bcet<=wcet violated'

	ntasks, nperiodic, nsporadic = len(period), tasktype.count('P'), tasktype.count('S') # count the tasks
	jitter = [0 if (tasktype[i]=='S' and jitter[i]>0) else jitter[i] for i in range(ntasks)] # force jitter=0 for sporadic tasks
	jobs = [-(-(horizon - offset[i])//period[i]) for i in range(len(period))] # double negation for managing '//' which rounds down

def controlsafetyproperty(whichmode):
	safetyconstraints = []
	my_resolution = 0
	for cid in range(len(A)): # for each controller
		for dimm in range(len(A[cid])-1): # for each dimension except the last one, which is u
			for jid in range(1,len(x[cid][0])):
				if jid==(lasso2//period[cid]): # and whichmode=='check': 
					# checking convergence over all dims: since steady-state value is 0, positive should decrease and negative should inc 
					# ~ if dimm==0:
					# ~ safetyconstraints += [And(x[cid][dimm][(lasso1//period[cid])]>0, x[cid][dimm][(lasso2//period[cid])]>x[cid][dimm][(lasso1//period[cid])])]
					# ~ safetyconstraints += [And(x[cid][dimm][(lasso1//period[cid])]<=0, x[cid][dimm][(lasso2//period[cid])]<x[cid][dimm][(lasso1//period[cid])])]
					safetyconstraints += [x[cid][dimm][(lasso2//period[cid])]>initstate[cid][dimm][1], x[cid][dimm][(lasso2//period[cid])]<initstate[cid][dimm][0]]
					# ~ else: pass
				elif dimm==0: # safety bounds applicable only for 0th dimension
					safetyconstraints += [x[cid][dimm][jid] > upperbound[cid][jid] + my_resolution] # upper bound violation
					safetyconstraints += [x[cid][dimm][jid] < lowerbound[cid][jid] - my_resolution] # lower bound
	if whichmode == 'check': safetyconstraints = Or(safetyconstraints) # assert for violation
	elif whichmode == 'guess': safetyconstraints = And([Not(sc) for sc in safetyconstraints]) # get some run
	#logging.info('constraints for '+whichmode+': '+str(safetyconstraints))
	smtsolver.add(safetyconstraints)
	
def guessdropset(dhat): # dhat = [[c1,c2],[c3,c4,c5]] : And(Or(And(c1),And(c2)),Or(And(c3)...))
	logging.info('running GuessDropSet...') # dhat= '+str(dhat))
	smtsolver.push() # prepare for a guess call to Z3
	controlsafetyproperty('guess') # insert safety constraints
	if len(dhat)>1: smtsolver.add(And([d1 for d1 in dhat]))
	elif len(dhat)==1: smtsolver.add(dhat[0])
	res = smtsolver.check()
	if res==unsat or res==unknown: 
		logging.info('GuessDropSet returned '+str(res)+', system has no feasible runs, repair is not possible, quitting')
		sys.exit()
	witness = smtsolver.model()
	for cid in range(len(A)):
		for dim in range(len(B[cid])):
			safeguessrun = [witness[x[cid][dim][i]].numerator().as_long()/witness[x[cid][dim][i]].denominator().as_long() for i in range(len(x[cid][dim]))]
			#logging.info('sys '+str(cid)+', dim '+str(dim)+', safeguessrun: '+str(safeguessrun))
	tr=getsortedtrace(witness)
	dset = [d[t1][j1] for (t1,j1,r1,s1,e1,d1,chain1,hit1,drop1) in tr if (drop1)]  # pick only drops
	undropset = [d[t1][j1] for (t1,j1,r1,s1,e1,d1,chain1,hit1,drop1) in tr if not drop1]  # pick non-drops
	#logging.info('GuessDropSet returned dropset = '+str(dset))
	smtsolver.pop() # restore context
	return (dset,undropset) # 2 choices for drops set: drop+hit OR only drop

def checkproperty(drops,undrops):
	logging.info('running CheckProperty...')
	smtsolver.push() # prepare for a check call to Z3
	workconservation() # insert WC constraints 
	controlsafetyproperty('check')
	smtsolver.add(drops) # set drop vars to True
	smtsolver.add([Not(u) for u in undrops]) # constrain these to be not dropped
	res = smtsolver.check()
	assert not res==unknown, 'SMT solver returned UNEXPECTED, quitting'
	if res==unsat:  
		logging.info('check call returned unsat, there is no safety violation. Synthesized PATCH = '+str(drops)+', for horizon = '+str(horizon))
		sys.exit()
	logging.info('control safety property violated')
	trmodel = smtsolver.model()
	tr=getsortedtrace(trmodel)
	printtrace(tr, 'printing task run:')	
	smtsolver.pop() # restore context
	# ~ return (trmodel,tr)
	return (trmodel)

def workconservation(): # work conservation constraints
	for tt in range(ntasks):
		for jj in range(jobs[tt]):
			clist = iset[(tt,jj)] + predset[(tt,jj)] # track chaining jobs (WC-part1)
			# ~ if len(clist)>1: smtsolver.add(ch[tt][jj]==Or([e[tp][jp]==s[tt][jj] for (tp,jp) in clist])) 
			if len(clist)>1: smtsolver.add(ch[tt][jj]==Or([And(e[tp][jp]==s[tt][jj],Not(d[tp][jp]),hit[tp][jp]) for (tp,jp) in clist])) 
			elif len(clist)==1: smtsolver.add(ch[tt][jj]==(And(e[clist[0][0]][clist[0][1]]==s[tt][jj], Not(d[clist[0][0]][clist[0][1]]), hit[clist[0][0]][clist[0][1]]) )) # only 1 pred
			else: smtsolver.add(Not(ch[tt][jj])) # no predecessor! hence negate
			
			smtsolver.add(Implies(r[tt][jj]<s[tt][jj],ch[tt][jj])) # wait => chain (WC-part2)
			
			slist = iset[(tt,jj)] + succset[(tt,jj)] # track who is scheduled after me (WC part3)
			if len(slist)>0: smtsolver.add([Implies(And(Not(ch[tt][jj]),e[tt][jj]<=s[ts][js]), s[tt][jj]<=r[ts][js]) for (ts,js) in slist]) 



def flipper(cid,hm,ctrdev,cemodell): # returns list of must jobs
	#logging.info('starting Flipper...')
	assert ((False,False) in hm) or (True in [i[1] for i in hm]), 'nothing to flip! for controller '+str(cid) # there should be atleast 1 miss or drop for flipping
	mustj = [] # must/critical jobs
	#logging.info('hit/miss and drop sequence: '+str(hm))
	propviolation = ctrdev.index(True) # get the 1st instance of safety violation
	# ~ propviolation = len(ctrdev)-list(reversed(ctrdev)).index(True)-1 # get the last instance
	#logging.info('property violation index = '+str(propviolation))
	
	allmisses = [i for (i,(hitt,dropp)) in enumerate(hm[:propviolation+1]) if ((not hitt) or dropp)] # pick all miss locations
	assert allmisses, 'unexpected! no miss/drop detected before property violation location '+str(propviolation)+' for control system '+str(cid)
	#logging.info('indices of all deadline misses = '+str(allmisses))
	
	for klen in range(1,len(allmisses)+1): 
		#logging.info('flipping all '+str(klen)+'-len misses to hits and simulating...')
		klenlist = list(combinations(allmisses,klen))  # get all k-len subsequences
		for klentuple in klenlist: # for each k-len subseq
			#logging.info('klentuple = '+str(klentuple))
			hmflip = copy.deepcopy(hm)
			for eachmiss in klentuple: 
				hmflip[eachmiss]=(True,False) # flipped miss->hit, drop->false i.e. 0->1 in the tuple
			#logging.info('flipped hit/miss seq: '+str(hmflip))
			
			xsim = [] # store the states
			xsim.append([cemodell[x[cid][d][0]].numerator().as_long()/cemodell[x[cid][d][0]].denominator().as_long() for d in range(len(B[cid]))]) # initial state
			for k in range(jobs[cid]): # x_k=Ax_k-1 + Bu_k-1, u_k-1=Kx_k-2; # simulator
				xsimkplus1 = []
				for dim in range(len(B[cid])):
					Ax_k = sum([A[cid][dim][i]*xsim[-1][i] for i in range(len(B[cid])) ])
					if (hmflip[k][0] and (not hmflip[k][1])): # hit and not drop
						u_k = R[cid][0] - sum([K[cid][i]*xsim[-1][i] for i in range(len(B[cid]))])
						Bu_k = B[cid][dim] * u_k
						xsimkplus1.append(Ax_k+Bu_k)
					else: # deadline miss
						if strategy=='zero-and-kill':
							if dim==len(B[cid])-1: xsimkplus1.append(0.0) # set u=0
							else: xsimkplus1.append(Ax_k)
						elif strategy=='hold-and-kill':
							if dim==len(B[cid])-1: xsimkplus1.append(xsim[-1][-1]) # set u_k=u_k-1
							else:
								Bu_k_hold_and_kill = B[cid][dim]*xsim[-1][-1]
								xsimkplus1.append(Ax_k+Bu_k_hold_and_kill)
						else: assert False, 'bug'
				xsim.append(xsimkplus1) # all states computed
			#logging.info('simulating sys '+str(cid)+': '+str(xsim))
			
			xsimviolations = []	 # identify violations, if any
			for jid in range(len(xsim)): # for each state vector
				tmp1 = []
				for dimm in range(len(xsim[jid])-1): # for each dim except the last
					if jid==(lasso2//period[cid]): # last state, check divergence
						tmp1.append(xsim[jid][dimm] < initstate[cid][dimm][0] or xsim[jid][dimm] > initstate[cid][dimm][1])
						# ~ if xsim[(lasso1//period[cid])][dimm]>0: tmp1.append(xsim[(lasso1//period[cid])][dimm] < xsim[(lasso2//period[cid])][dimm])
						# ~ else: tmp1.append(xsim[(lasso2//period[cid])][dimm] < xsim[(lasso1//period[cid])][dimm])
					elif dimm==0: # check safety violation
						tmp1.append(xsim[jid][dimm] > upperbound[cid][jid] or xsim[jid][dimm] < lowerbound[cid][jid])
					else: # ignore for other dims
						tmp1.append(False)
				xsimviolations.append(tmp1)

			# ~ for v in range(len(xsim[:-1])): # examine violations at each step except the last
				# ~ xsimviolations.append(xsim[v][0]>upperbound[cid][v] or xsim[v][0]<lowerbound[cid][v]) # examine violations in 0th dim except the last (u)
			# ~ if xsim[(lasso1//period[cid])][0]>0: xsimviolations.append(xsim[(lasso2//period[cid])][0] > xsim[(lasso1//period[cid])][0]) # the last state, lasso2, check divergence
			# ~ else: xsimviolations.append(xsim[(lasso2//period[cid])][0] < xsim[(lasso1//period[cid])][0])
			assert len(xsimviolations)==len(xsim)
			#logging.info('violations for all dimensions: '+str(xsimviolations))
			if not (True in xsimviolations[propviolation]): #  no violation for any dim
			# ~ if not (True in [True in x for x in xsimviolations]):#[propviolation]): #  no violation anywhere
				#logging.info('flipping restored control safety')
				for eachmiss in klentuple: 
					mustj.append((cid,eachmiss)) #(hm[loc][0],hm[loc][1])) # pick each job from the critical subsequence 

		if mustj: 
			#logging.info('at k-len = '+str(klen)+', mustj = '+str(mustj))
			return mustj # we got must jobs at this k-level 
	
	assert False, 'empty mustj! for controller '+str(cid)+' after trying all k-len flips, exiting...' 

def old_flipper(cid,hm,ctrdev,cemodell): 
	for loc in allmisses: # try flip 0->1 # only 1-flip here # tuple: (hit1,drop1)  
		xsim = [[0.0]*len(B[cid])]*len(ctrdev) # initialize for new simulation
		xsim[0] = [cemodell[x[cid][i][0]].numerator().as_long()/cemodell[x[cid][i][0]].denominator().as_long() for i in range(len(B[cid]))]
		usim = [0.0]*(len(hm)+1) # init u_0
		hmflip = copy.deepcopy(hm)
		hmflip[loc]=(True,False) # flipped False->True,drop->false i.e. 0->1 in the tuple
		for k in range(1,len(xsim)): # x_k=Ax_k-1 + Bu_k-1, u_k-1=Kx_k-2; # simulator
			mtmp=[]
			for m in range(len(xsim[0])):
				if (not hmflip[k-1][0]) or hmflip[k-1][1]: # not hit or drop
					if strategy=='zero-and-kill': usim[k] = 0.0
					else: usim[k] = usim[k-1]
				else: usim[k] = R[cid][0] - sum([K[cid][n]*xsim[k-1][n] for n in range(len(K[cid]))]) # u_k = R-Kx_k-1
				mtmp.append(sum([A[cid][m][n]*xsim[k-1][n] for n in range(len(B[cid]))]) + B[cid][m]*usim[k]) # 
			xsim[k]=mtmp # this roundabout mtmp append is required
		if (not (xsim[propviolation][0]-idealtrace[cid][propviolation]>maxdev[cid])) and (not (idealtrace[cid][propviolation]>maxdev[cid]-xsim[propviolation][0]>maxdev[cid])):
			mustj.append((cid,loc)) #(hm[loc][0],hm[loc][1])) # pick the critical job

	#logging.info('mustj = '+str(mustj))
	assert mustj, 'empty mustj! for controller '+str(cid)+', 1-flip did not yield property safety, exiting...' 
	return mustj

def closure(mustjobs,cemodel):
	logging.info('running Closure...')
	tr = getsortedtrace(cemodel) # extract trace from the solver witness
	mayset = set([i for i in tr if ((i[0],i[1]) in mustjobs)]) # init to the mustjobs
	Mminus = set([i for i in mayset if i[8]]) # get critical ones that were dropped
	maysetcopy = copy.deepcopy(mayset) # store a copy
	assert mayset, 'empty mayset!'
	candidates = set([i for i in tr if (i[0],i[1]) not in mustjobs]) # pick the rest of the trace
	assert candidates, 'empty candidates!'
	while True:
		foundoverlap = False
		for (t1,j1,rr1,ss1,ee1,dd1,cc1,hh1,dr1) in candidates: 
			if (not hh1) or dr1: continue # this candidate is either miss or dropped, ignore
			for (t2,j2,rr2,ss2,ee2,dd2,cc2,hh2,dr2) in mayset:
				if dr2: continue # this mayjob was dropped, deal later
				if rr2 < ee1 and ee1 <= ss2:
					foundoverlap = True
					candidates.remove((t1,j1,rr1,ss1,ee1,dd1,cc1,hh1,dr1))
					mayset.add((t1,j1,rr1,ss1,ee1,dd1,cc1,hh1,dr1))
					break
			if foundoverlap: break # iterate all over again from while
		if not foundoverlap: 
			dropcandidates = mayset.difference(maysetcopy)
			#logging.info('dropcandidates = '+str([(i[0],i[1]) for i in dropcandidates]))
			
			tmp1,tmp2 = list(dropcandidates), list(Mminus)
			tmp3 = [d[i[0]][i[1]] for i in tmp1] + [Not(d[i[0]][i[1]]) for i in tmp2]
			if len(tmp3) > 1: tmp3 = Or(tmp3)
			elif len(tmp3)==1: tmp3 = tmp3[0]
			#logging.info('closure OR clause ='+str(tmp3))
			return tmp3

# main starts
logging.basicConfig(format='%(asctime)s %(levelname)-5s: %(message)s',level='INFO',datefmt='%H:%M:%S')
logging.info('starting PATCHER: scheduler patch synthesis tool, ver 0.1')
systemspec() # load the system
logging.info('found '+str(len(period))+' tasks, '+str(sum(jobs))+' jobs')
smtsolver=Solver() # instantiate the solver
set_option(rational_to_decimal=True)
## change the solver precision as required
set_option(precision=2) # TBD evaluate this parameter's impact on performance

r = [[Int('r'+str(tid)+'@'+str(jid)) for jid in range(jobs[tid])] for tid in range(ntasks)] # release time
smtsolver.add([And(j1*period[t1]+offset[t1]<=r[t1][j1], r[t1][j1]<=j1*period[t1]+offset[t1]+jitter[t1]) for t1 in range(ntasks) for j1 in range(jobs[t1]) if tasktype[t1]=='P']) # set up release times
smtsolver.add([r[t1][j1]-r[t1][j1-1]>=period[t1] for t1 in range(ntasks) for j1 in range(1,jobs[t1]) if tasktype[t1]=='S']) # min separation between sporadic jobs
smtsolver.add([r[t1][0]>=offset[t1] for t1 in range(ntasks) if tasktype[t1]=='S']) 

s = [[Int('s'+str(tid)+'@'+str(jid)) for jid in range(jobs[tid])] for tid in range(ntasks)] # start time
e = [[Int('e'+str(tid)+'@'+str(jid)) for jid in range(jobs[tid])] for tid in range(ntasks)] # end time
smtsolver.add([rr<=ss for (x,y) in zip(r,s) for (rr,ss) in zip(x,y)])
smtsolver.add([e[t1][j1]<=s[t1][j1+1] for t1 in range(len(s)) for j1 in range(len(s[t1])-1)]) # intra-task precedence

d = [[Bool('d'+str(tid)+'@'+str(jid)) for jid in range(jobs[tid])] for tid in range(ntasks)] 
smtsolver.add([Not(d[tid][jid]) for tid in range(ntasks) for jid in range(jobs[tid]) if tasktype[tid]=='S']) # drops are only for periodic tasks

hit = [[Bool('hit'+str(tid)+'@'+str(jid)) for jid in range(jobs[tid])] for tid in range(ntasks)] # deadline hit/miss 
smtsolver.add([hit[t1][j1] == (s[t1][j1]+wcet[t1]<=(j1+1)*period[t1]+offset[t1]) for t1 in range(ntasks) for j1 in range(jobs[t1]) if tasktype[t1]=='P']) # hit == s+wcet<=deadline

smtsolver.add([Implies(Or(d[t1][j1],Not(hit[t1][j1])),s[t1][j1]==e[t1][j1]) for t1 in range(ntasks) for j1 in range(jobs[t1]) if tasktype[t1]=='P']) # drop OR Not(hit) => s==e 
smtsolver.add([Implies(And(Not(d[t1][j1]),hit[t1][j1]), And(s[t1][j1]+bcet[t1]<=e[t1][j1], e[t1][j1]<=s[t1][j1]+wcet[t1])) for t1 in range(ntasks) for j1 in range(jobs[t1]) if tasktype[t1]=='P']) # Not(drop) OR Not(hit) => bcet <= e-s <= wcet 
smtsolver.add([And(s[t1][j1]+bcet[t1]<=e[t1][j1], e[t1][j1]<=s[t1][j1]+wcet[t1]) for t1 in range(ntasks) for j1 in range(jobs[t1]) if tasktype[t1]=='S']) # for sporadic, bcet <= e-s <= wcet 

iset = {(tt,jj):[] for tt in range(ntasks) for jj in range(jobs[tt])} # interference set init
predset = {(tt,jj):[(tt,jj-1)] for tt in range(ntasks) for jj in range(1,jobs[tt])} # predecessor set init
succset = {(tt,jj):[(tt,jj+1)] for tt in range(ntasks) for jj in range(jobs[tt]-1)} # successor set init
for tt in range(ntasks): # rest of the initilization
	predset[(tt,0)] = []
	succset[(tt,jobs[tt]-1)] = []
		
for t1 in range(ntasks): # inter-task static precedence constraints
	for j1 in range(jobs[t1]-1,-1,-1): # build inter-task edges	# range(start,stop,step) in reverse order
		for t2 in range(ntasks): # (t1,j1) -> (t2,j2)
			# ~ if t1==t2 or tasktype[t1]=='S': continue # move to next task, no precedence here
			if t1==t2: continue # move to next task, no precedence here
			for j2 in range(jobs[t2]):
				if staticprecedence((t1,j1),(t2,j2)): # 
					smtsolver.add(e[t1][j1] <= s[t2][j2]) # uniproc
					succset[(t1,j1)].append((t2,j2)) # update successor list
					predset[(t2,j2)].append((t1,j1)) # update predecessor list
					break # we paired the furthest possible job of t1 with the earliest possible job of t2
				else: 
					if staticprecedence((t2,j2),(t1,j1)): pass
					else: iset[(t1,j1)].append((t2,j2)) # append job2 # job2 interferes with job1

for (t1,j1) in iset.keys(): # ensure scheduling order between interfering jobs
	for (t2,j2) in iset[(t1,j1)]: # TBD: overlap and scheduling based on hit/drop 
		dl1, dl2 = period[t1]*(j1+1)+offset[t1], period[t2]*(j2+1)+offset[t2]
		if tasktype[t1]=='P' and tasktype[t2]=='P': 
			smtsolver.add(Implies(e[t1][j1]<=s[t2][j2], Or(dl1<=dl2, s[t1][j1]<r[t2][j2])))
		elif tasktype[t1]=='P' and tasktype[t2]=='S': 
			smtsolver.add(Implies(e[t1][j1]<=s[t2][j2], Or(dl1<=s[t2][j2]+period[t2], s[t1][j1]<r[t2][j2])))
		elif tasktype[t1]=='S' and tasktype[t2]=='P': 
			smtsolver.add(Implies(e[t1][j1]<=s[t2][j2], Or(s[t1][j1]+period[t1]<=dl2, s[t1][j1]<r[t2][j2])))
		else: 
			smtsolver.add(Implies(e[t1][j1]<=s[t2][j2], Or(s[t1][j1]+period[t1]<=s[t2][j2]+period[t2], s[t1][j1]<r[t2][j2])))

smtsolver.add([Or(e[t1][j1]<=s[t2][j2],e[t2][j2]<=s[t1][j1]) for (t1,j1) in iset.keys() for (t2,j2) in iset[(t1,j1)]]) # ensure non-overlap between interfering jobs

ch = [[Bool('ch'+str(tid)+'@'+str(jid)) for jid in range(jobs[tid])] for tid in range(ntasks)] # chain var

# x@c@d@n c=controller,d=dimension,n=step
# this is the augmented state space, where x[c][-1][*] corresponds to control input u
x = [[[Real('x'+str(c)+'@'+str(d)+'@'+str(n)) for n in range(jobs[c]+1)] for d in range(len(B[c]))] for c in range(len(B))]  
smtsolver.add([And(initstate[c][d][0]<=x[c][d][0], x[c][d][0]<=initstate[c][d][1]) for c in range(len(initstate)) for d in range(len(initstate[c]))]) # lowerbound <= x[c][d][0] <= upperbound init box 

# x_k+1 = A.x_k + B.u_k, u_k = R-Kx_k # adjusted for augmented control system
for c in range(len(A)): # for each controller 
	for n in range(jobs[c]):  # for each step of evolution
		for dim in range(len(B[c])): # for each dimension
			Ax_k = [ A[c][dim][i]*x[c][i][n] for i in range(len(B[c])) ]
			u_k = R[c][0] - Sum([ K[c][i]*x[c][i][n] for i in range(len(B[c])) ])
			Bu_k = [ B[c][dim]*u_k ]
			smtsolver.add(Implies(And(hit[c][n],Not(d[c][n])), x[c][dim][n+1]==Sum(Ax_k+Bu_k))) 
			if strategy=='zero-and-kill':
				smtsolver.add(Implies(Not(And(hit[c][n],Not(d[c][n]))), x[c][dim][n+1]==Sum(Ax_k))) 
				if dim==len(B[c])-1: # set u=0
					smtsolver.add(Implies(Not(And(hit[c][n],Not(d[c][n]))), x[c][-1][n+1]==0))
			elif strategy=='hold-and-kill':
				Bu_k_hold_and_kill = [ B[c][dim]*x[c][-1][n] ]
				smtsolver.add(Implies(Not(And(hit[c][n],Not(d[c][n]))), x[c][dim][n+1]==Sum(Ax_k+Bu_k_hold_and_kill))) 
				if dim==len(B[c])-1: # set u_k==u_k-1
					smtsolver.add(Implies(Not(And(hit[c][n],Not(d[c][n]))), x[c][-1][n+1]==x[c][-1][n]))
			else: assert False, 'unexpected bug'
				
nref = 0 
drophat = []
# ~ drophat += [d[cid][(lasso1//period[cid])-1-k]==d[cid][(lasso2//period[cid])-1-k] for cid in range(len(A)) for k in range((lasso2//period[cid])-(lasso1//period[cid]))] # duplicate flags, with stem+lasso1+lasso2
# ~ drophat += [hit[cid][(lasso1//period[cid])-1-k]==hit[cid][(lasso2//period[cid])-1-k] for cid in range(len(A)) for k in range((lasso2//period[cid])-(lasso1//period[cid]))] # duplicate hit EXPT, hoping to prevent empty lasso drop sets
while True:
	nref += 1
	logging.info('====PATCHER: starting refinement iteration '+str(nref))
	dropset,undropset = guessdropset(drophat)
	cemodel = checkproperty(dropset,undropset) 
	(controllerid,hitseq,ctracedev) = gethitsequence(cemodel)
	mustjobs = flipper(controllerid,hitseq,ctracedev,cemodel)
	mayjobs = closure(mustjobs,cemodel) # returns an OR clause
	drophat.append(mayjobs) # 
	
