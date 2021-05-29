"""
Code to test the derivative calculation I wrote (see p. 2-2-1).

Network topology is on page 2-2-3.

This code is ugly and only meant for prototyping, should be treated
with extreme caution.
"""

# Parameters
totalDemand = 50          # q

goodLotCapacity = 10      # C
goodLotDifficulty = 10    # lambda
goodLotSearchCost = 1

badLotParkCost = 4

cycleCost = 2

epsilon = 0.000001 # Termination criterion for network loading
UEThreshold = 0.01 # Any solution with smaller gap than this is considered UE

import math
import pdb
import numpy as np

def parkingFlow(flow, capacity, difficulty):
	assert (capacity <= difficulty), "Lot capacity cannot exceed difficulty."
	assert (flow >= 0 and capacity >= 0 and difficulty >= 0), "Invalid params."
	return (math.exp(-flow / difficulty))*flow
 
# Not including strong feasibility check, since any solution with 
# cycleFraction < 1 should be strongly feasible.
def networkLoading(searchFraction, cycleFraction):
	linkFlows = { 'goodLotSearch': 0,
				  'goodLotSkip': 0,
				  'goodLotSuccess': 0,
				  'goodLotFail': 0,
				  'cycleBack': 0,
				  'badLot': 0
				}
	unparked = { 'origin': totalDemand,
				 'cyclePoint': 0,
				}


	while sum(unparked.values()) > epsilon:
		# First process origin...
		#print(unparked)
		# if searchFraction >0  and cycleFraction>0:
		# 	pdb.set_trace()
		newSearching = unparked['origin'] * searchFraction
		newNotSearching = unparked['origin'] - newSearching
		linkFlows['goodLotSearch'] += newSearching
		unparked['cyclePoint'] += newNotSearching
		linkFlows['goodLotSkip'] += newNotSearching
		unparked['origin'] -= newSearching + newNotSearching

		# ... then split at parking lot
		newTotalSuccess = parkingFlow(linkFlows['goodLotSearch'], 
									  goodLotCapacity, goodLotDifficulty)
		newTotalFail = linkFlows['goodLotSearch'] - newTotalSuccess
		unparked['cyclePoint'] += newTotalFail - linkFlows['goodLotFail']
		linkFlows['goodLotFail'] = newTotalFail
		linkFlows['goodLotSuccess'] = newTotalSuccess
		
		# ...then process return point.
		cyclingBack = unparked['cyclePoint'] * cycleFraction
		badParking = unparked['cyclePoint'] - cyclingBack
		linkFlows['cycleBack'] += cyclingBack
		unparked['origin'] += cyclingBack
		linkFlows['badLot'] += badParking
		unparked['cyclePoint'] = 0


	return linkFlows
	

# def networkLoadingCan(searchFraction, cycleFraction):
# 	y = {'goodLotFail':0
# 		}
# 	linkFlows = { 'goodLotSearch': 0,
# 				  'goodLotSkip': 0,
# 				  'goodLotSuccess': 0,
# 				  'goodLotFail': 0,
# 				  'cycleBack': 0,
# 				  'badLot': 0
# 				}
# 	unparked = { 'origin': totalDemand,
# 				 'cyclePoint': 0,
# 				}

# 	while sum(unparked.values()) > epsilon:
# 		linkFlows['goodLotSkip'] +=  (1-searchFraction)*unparked['origin']
# 		unparked['cyclePoint'] += (1-searchFraction)*unparked['origin']
# 		unparked['origin'] -= (1-searchFraction)*unparked['origin']
		
# 		linkFlows['cycleBack'] += cycleFraction*unparked['cyclePoint']
# 		unparked['origin'] += cycleFraction*unparked['cyclePoint']
# 		unparked['cyclePoint'] -= cycleFraction*unparked['cyclePoint']
# 		######

# 		y['goodLotFail'] = linkFlows['goodLotFail']
# 		linkFlows['goodLotSearch'] += searchFraction*unparked['origin']
# 		before = linkFlows['goodLotSuccess']
# 		linkFlows['goodLotSuccess'] = (1 - math.exp(-linkFlows['goodLotSearch'] / goodLotDifficulty))*linkFlows['goodLotSearch']
# 		linkFlows['goodLotFail'] = linkFlows['goodLotSearch'] - linkFlows['goodLotSuccess']
# 		unparked['cyclePoint'] += linkFlows['goodLotFail'] - y['goodLotFail']
# 		unparked['origin'] -= linkFlows['goodLotSuccess'] - before
# 		linkFlows['badLot'] += (1-cycleFraction)*unparked['cyclePoint']
# 		unparked['cyclePoint'] -= (1-cycleFraction)*unparked['cyclePoint']

# 	return linkFlows

def totalCost(linkFlows):
	return (goodLotSearchCost * linkFlows['goodLotSearch']
			+ badLotParkCost * linkFlows['badLot']
			+ cycleCost * linkFlows['cycleBack'])
			
# See 2-2-3 for derivation
def costLabels(linkFlows, searchFraction, cycleFraction):
	label = dict()
	if linkFlows['goodLotSearch'] > 0:
		successProb = linkFlows['goodLotSuccess'] / linkFlows['goodLotSearch']
	else:
		successProb = 1
	seekSuccessProb = searchFraction * successProb
	label['origin'] = ((searchFraction * goodLotSearchCost
					   + (1-seekSuccessProb) 
						  * badLotParkCost * (1-cycleFraction))
					  / (1 - (1 - seekSuccessProb) * cycleFraction))
	label['cyclePoint'] = badLotParkCost * (1 - cycleFraction) + \
							label['origin'] * cycleFraction
	return label
	
def gap(costLabels, linkFlows, searchFraction, cycleFraction):
	# First at origin...
	if linkFlows['goodLotSearch'] > 0:
		successProb = linkFlows['goodLotSuccess'] / linkFlows['goodLotSearch']
	else:
		successProb = 1
	goodLotSearchLabel = goodLotSearchCost + \
						(1-successProb) * costLabels['cyclePoint']
	goodLotSkipLabel = costLabels['cyclePoint']
	goodLotSearchGap = max(goodLotSearchLabel - goodLotSkipLabel, 0)
	goodLotSkipGap = max(goodLotSkipLabel - goodLotSearchLabel, 0)    
	originGap = searchFraction * goodLotSearchGap  \
			   + (1 - searchFraction) * goodLotSkipGap
			   
	cycleLabel = costLabels['origin']
	cycleGap = max(cycleLabel - badLotParkCost, 0)
	badParkGap = max(badLotParkCost - cycleLabel, 0)
	cyclePointGap = cycleFraction * cycleGap  \
				   + (1 - cycleFraction) * badParkGap
				   
	return originGap + cyclePointGap                   



def findUESO(gridPoints):
	"""
	Do a grid search (avoiding 1.0 for strong feasibility) to find UE and SO
	"""
	SO = dict()
	UE = dict()
	UE['gap'] = 9999
	SO['cost'] = 9999
	for x in range(gridPoints):
		searchProb = x / gridPoints    
		for y in range(gridPoints):
			print(x,y)
			cycleProb = y / gridPoints
			linkFlows = networkLoading(searchProb, cycleProb)
			labels = costLabels(linkFlows, searchProb, cycleProb)
			cost = totalCost(linkFlows)
			gapVal = gap(labels, linkFlows, searchProb, cycleProb)
			# Among the UE solutions, try to find one with low cost
			if UEThreshold <= gapVal < UE['gap'] or (gapVal < UEThreshold and cost < UE['cost']): # Better UE (havent found ue yet)
				UE['search fraction'] = searchProb
				UE['cycle fraction'] = cycleProb
				UE['link flows'] = linkFlows
				UE['labels'] = labels
				UE['cost'] = cost
				UE['gap'] = gapVal            
				UEGap = gapVal
			if cost < SO['cost']:
				SO['search fraction'] = searchProb
				SO['cycle fraction'] = cycleProb
				SO['link flows'] = linkFlows
				SO['labels'] = labels
				SO['cost'] = cost
				SO['gap'] = gapVal
				SOCost = cost
	return (UE, SO)

def initialTest(sf=0.5, cf=0.5):
	print("Basic test")
	linkFlows = networkLoading(sf, cf)
	labels = costLabels(linkFlows, sf, cf)
	print(linkFlows)
	print(totalCost(linkFlows))
	print(labels)
	print(gap(labels, linkFlows, sf, cf))
	
def UESO_Test():    
	UE, SO = findUESO(1000)
	print("UE solution:")
	print(UE)
	print("SO solution:")    
	print(SO)

def derivativeTest():
	epsilon = 0.001
	baseSearch = 0.5
	baseCycle = 0.5
	originalFlows = networkLoading(baseSearch, baseCycle)
	dSearchFlows = networkLoading(baseSearch + epsilon, baseCycle)
	dCycleFlows = networkLoading(baseSearch, baseCycle + epsilon)
	derSearch = { ij: (dSearchFlows[ij] - originalFlows[ij]) / epsilon
				  for ij in originalFlows}
	derCycle = { ij: (dCycleFlows[ij] - originalFlows[ij]) / epsilon
				  for ij in originalFlows}
	return derSearch, derCycle
			
def get_derivatives(linkFlows, searchFraction, cycleFraction):
	x_1 = linkFlows['goodLotSearch']
	p_1 = (math.exp(-x_1/10))
	p_1_der = (-1.0/10)*(math.exp(-x_1/10))

	
	A = np.matrix([[1, 0, 0, 0, -searchFraction, 0], 
				  [0, 1, 0, 0, -(1 - searchFraction), 0], 
				  [-(p_1 + p_1_der*x_1), 0, 1, 0, 0, 0],
				  [-1, 0, 1, 1, 0, 0],
				  [0, -cycleFraction, 0, -cycleFraction, 1, 0],
				  [0, -(1-cycleFraction), 0, -(1-cycleFraction), 0, 1]])


	b1 = [totalDemand + linkFlows['cycleBack'], 0, 0, 0, 0, 0]
	b2 = [0, totalDemand + linkFlows['cycleBack'], 0, 0, 0, 0]
	b5 = [0, 0, 0, 0, linkFlows['goodLotSkip'] + linkFlows['goodLotFail'], 0]
	b6 = [0, 0, 0, 0, 0, linkFlows['goodLotSkip'] + linkFlows['goodLotFail']]

	Jx_ind = np.zeros((6, 4))

	A_inv = np.linalg.inv(A)
	Jx_ind[:, 0] = np.dot(A_inv, b1)
	Jx_ind[:, 1] = np.dot(A_inv, b2)
	Jx_ind[:, 2] = np.dot(A_inv, b5)
	Jx_ind[:, 3] = np.dot(A_inv, b6)

	# print(Jx_ind)
	# Jx_feasibility = np.zeros((6, 2))
	# Jx_feasibility[:, 0] = Jx_ind[:,0] - Jx_ind[:,1]
	# Jx_feasibility[:, 1] = Jx_ind[:,2] - Jx_ind[:,3]
	# print(Jx_feasibility)
	return Jx_ind

import copy

def clip(fraction):
	return np.clip(fraction, 0, 1)

def descent_test(beta=0.2, sigma=0.01, s=0.2):
	
	searchFraction, cycleFraction = init_fractions()
	eps = 1e-5
	orig_cost = 100000
	new_cost = 0 
	new_cost_list = []
	i = 0
	consistent = False

	while not consistent:

		if i>=5:
			if orig_cost - new_cost < eps and abs(orig_cost - new_cost_list[-5]) < eps:
				consistent = True

		
		linkFlows = networkLoading(searchFraction, cycleFraction)
		orig_sfrac = searchFraction
		orig_cfrac = cycleFraction
		orig_linkFlows = copy.deepcopy(linkFlows)
		orig_cost = totalCost(linkFlows)
		print(orig_cost)
		grad_link_flows_ind = get_derivatives(linkFlows, searchFraction, cycleFraction)
		

		t_vec = np.array([1, 0, 0, 0, 2, 4])

		new_cost = 10000000
		step_size = s

		if i%2==0:
			coordinate = 'srch'
			diff = (grad_link_flows_ind[:, 0] - grad_link_flows_ind[:, 1]).dot(t_vec)

			# if grad_link_flows_ind[:, 0].dot(t_vec) > grad_link_flows_ind[:, 1].dot(t_vec):
				# diff = grad_link_flows_ind[:, 0].dot(t_vec) - grad_link_flows_ind[:, 1].dot(t_vec)
			# else:
				# diff = grad_link_flows_ind[:, 1].dot(t_vec) - grad_link_flows_ind[:, 0].dot(t_vec)
		else:
			coordinate = 'cycle'
			diff = (grad_link_flows_ind[:, 2] - grad_link_flows_ind[:, 3]).dot(t_vec)
			
			# if grad_link_flows_ind[:, 2].dot(t_vec) > grad_link_flows_ind[:, 3].dot(t_vec):
				# diff = grad_link_flows_ind[:, 2].dot(t_vec) - grad_link_flows_ind[:, 3].dot(t_vec)
			# else:
				# diff = grad_link_flows_ind[:, 3].dot(t_vec) - grad_link_flows_ind[:, 2].dot(t_vec)


		while orig_cost - new_cost < -sigma*step_size*diff:
			
			searchFraction = orig_sfrac
			cycleFraction = orig_cfrac

			if coordinate == 'srch': #move along searchFrac			
				diff_btw = diff/(totalDemand + linkFlows['cycleBack']) 
				delta_frac = step_size*diff_btw
				searchFraction -= delta_frac
				searchFraction = clip(searchFraction) #projection

				# if grad_link_flows_ind[:, 1].dot(t_vec) < grad_link_flows_ind[:, 0].dot(t_vec):
				# 	searchFraction = max(0, searchFraction - delta_frac)
				# else:
				# 	searchFraction = min(1, searchFraction + delta_frac)

			else: #move along cycleFrac
				diff_btw = diff/(linkFlows['goodLotFail'] + linkFlows['goodLotSkip']) 
				delta_frac = step_size*diff_btw
				cycleFraction -= delta_frac
				cycleFraction = clip(cycleFraction) #projection

				# if grad_link_flows_ind[:, 3].dot(t_vec) < grad_link_flows_ind[:, 2].dot(t_vec):
				# 	cycleFraction = max(0, cycleFraction - delta_frac)
				# else:
				# 	cycleFraction = min(1, cycleFraction + delta_frac)


			linkFlows = networkLoading(searchFraction, cycleFraction)
			new_cost = totalCost(linkFlows)
			step_size *= beta
		print(searchFraction, cycleFraction)
		new_cost_list.append(new_cost)

		i += 1

	return new_cost, searchFraction, cycleFraction


def init_fractions():
	return 0.5, 0.5

if __name__ == '__main__':

	UESO_Test()
	initialTest()
	derSearch, derCycle = derivativeTest()
	print(derSearch)
	print(derCycle)

	so_obj, searchFraction, cycleFraction = descent_test()
	print('System Optimal cost: {} \n search fraction: {} \n cycle fraction: {}'.format(so_obj, searchFraction, cycleFraction))
