import argparse
import time
import heapq
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

class State(object):

    def __init__(self, tower_state):
        self.tower_state = tower_state
        
    def __hash__(self):
        return hash(self.__tuple__())

    def __tuple__(self):
        return (self.tower_state)

    def __lt__(self, other):
        return type(self) == type(other) and self.__tuple__() < other.__tuple__()

    def __gt__(self, other):
        return type(self) == type(other) and self.__tuple__() > other.__tuple__()

    def __le__(self, other):
        return type(self) == type(other) and self.__tuple__() <= other.__tuple__()

    def __ge__(self, other):
        return type(self) == type(other) and self.__tuple__() >= other.__tuple__()

    def __eq__(self, other):
        return type(self) == type(other) and self.__tuple__() == other.__tuple__()        
        
class Tower(object):
  

    def __init__(self, n, rods):
        #initializing the attributes of the class
        self.rods = rods
        self.discs = n
        self.rod_discs = [[] for i in range(rods)]
        self.rod_discs[0] = [i for i in range(n-1,-1,-1)]
        
    def get_disc_count(self):
        
        return self.discs
        
    def has_disc(self, rod, pos):
        return len(pos[rod]) > 0
        
    def is_valid_move(self,disc,rod, pos):
        
        return pos[rod][-1] > disc if self.has_disc(rod, pos) else True
    
    def get_rod(self, disc, pos):
        
        for rod in range(self.rods):
            if disc in pos[rod]:
                return rod
        return None
    
    def get_initial_state(self):
        
        return State(self.rod_discs)

    def get_successor_states(self, state):
        
        result = []
        #list of rod_discs from the class Tower world
        current_pos = state.tower_state
        
        for rod in range(self.rods):
            discs_on_rod = current_pos[rod]
            if len(discs_on_rod) > 0:
                
                disc_to_move = discs_on_rod[-1]
            else:
                continue
            for rod_next in range(self.rods):
                
                if rod != rod_next and self.is_valid_move(disc_to_move, rod_next, current_pos):
                    new_pos = current_pos[:]
                    
                    new_pos[rod_next] = current_pos[rod_next] + [disc_to_move]
                    new_pos[rod] = new_pos[rod][:-1]
                    
                    i_state = State(new_pos)
                    
                    result.append(i_state)
        return result
    
class SearchNode(object):
    
    def __init__(self, state, h, parent = None):
        #initializing the attributes of the class to find the states correctly
        #Here, the parent node is stored to keep track along with the h and g values 
        #At every level in the A* algo, the g value increases by 1
        assert(isinstance(state, State))
        assert(isinstance(h, int))
        
        assert(parent is None or isinstance(parent, SearchNode))
        self.state = state
        self.parent = parent
        self.h = h
        #the algorithm has unit cost 
        self.g = 0 if parent is None else parent.g + 1

    def extract_plan(self):
        plan = []
        p = self
        while p != None:
            plan.append(p.state)
            p = p.parent
        return list(reversed(plan))

    def set_flag(self, flags, val = True):
        flags[str(self.state.tower_state)] = True

    def check_if_flagged(self, flags):
        return flags[str(self.state.tower_state)]

    def get_f_value(self):
        return self.h + self.g
    
    def __lt__(self, other):
        return (self.get_f_value(), self.g, self.state) < (other.get_f_value(), other.g, other.state)

class SearchResult(object):

    def __init__(self, expansions = 0, visited = 0, plan = None):
        
        self.expansions = expansions
        self.visited = visited
        self.plan = plan
        


def astar_search(tower_state, heuristic):

    assert(isinstance(tower_state, Tower))
    assert(isinstance(heuristic, Heuristic))
    initial_state = tower_state.get_initial_state()
    disc_count = tower_state.get_disc_count()
    rod_count = 4
    open_list = [ SearchNode(initial_state, heuristic(initial_state)) ]
    expanded = defaultdict(lambda: False)
    visited = defaultdict(lambda: None)
    visited[str(initial_state.tower_state)] = open_list[0].h
    
    result = SearchResult()
            
    while len(open_list) > 0:
        node = heapq.heappop(open_list)
        if node.check_if_flagged(expanded):
            continue
        #For the node with the least amount of cost, expand it.
        print('Currently expanding node',str(node.state.tower_state),'with f =',node.get_f_value())
        result.expansions += 1
        if len(node.state.tower_state[rod_count-1]) == disc_count:
            result.plan = node.extract_plan()
            print('Goal state reached',str(node.state.tower_state))
            for rod in range(tower_state.rods):            
                print(str(rod), str(node.state.tower_state[rod]))
            break
        successor = tower_state.get_successor_states(node.state)
        for i in successor:
            if expanded[str(i.tower_state)]:
                continue
            if visited[str(i.tower_state)] is None:
                print('visiting',str(i.tower_state),'with h =',heuristic(i))
                result.visited += 1
                visited[str(i.tower_state)] = heuristic(i)
            heapq.heappush(open_list, SearchNode(i, visited[str(i.tower_state)], node))
            
       
        node.set_flag(expanded)
        
    return result

class Heuristic(object):

    def __init__(self, tower_state):
        assert(isinstance(tower_state, Tower))
        self.tower_state = tower_state
        self.disc_count = tower_state.get_disc_count()
        self.rod_count = 4
        

    def __call__(self, state):
        assert(isinstance(state, State))

            
    
class ManhatdistancefromlastpegHeuristic(Heuristic):
      '''
      The code will return the manhatten distance from the last peg for the topmost disk.
    '''
      def __init__(self, tower_state):
          super(ManhatdistancefromlastpegHeuristic, self).__init__(tower_state)
          
      def __call__(self, state):
          assert(isinstance(state, State))
          
          disc =  state.tower_state
          min_disc = []
          for i in range(len(disc)):
              if len(disc[i])>0:
                  min_disc.append(min(disc[i]))
              else:
                  min_disc.append(0)
        
          manhatt = 0
          for i in min_disc:
              for j in min_disc:
                  if i != j:
                      manhatt = abs(i - self.rod_count) + abs(j-self.rod_count)
          return manhatt
              
              
       
          
         
        
class EucldistancefromlastpegHeuristic(Heuristic):
    '''
    The code will return the minumum distance of the larger disc from the
    last peg.
    '''
    def __init__(self, tower_state):
        super(EucldistancefromlastpegHeuristic, self).__init__(tower_state)
        
        
    def __call__(self, state):
        assert(isinstance(state, State))
        # The actual heuristic computation.
        
        disc =  state.tower_state
        max_disc = []
        for i in range(len(disc)):
            if len(disc[i])>0:
                max_disc.append(max(disc[i]))
            else:
                max_disc.append(0)
       
        euclidean=[]
        for i in max_disc:
            
            euclidean.append(int(np.sqrt(np.sum(i-self.rod_count)**2)))
            return min(euclidean)
         
        
    
class BlindHeuristic(Heuristic):
    """
    The blind heuristic. Returns 0 for every state. Using this heuristic will
    turn A* into simple Dijkstra search. In our unit-cost setting, A* with the
    blind heuristic boils down to a simple breadth-first search.
    """

    def __init__(self, tower_state):
        # Call the super constructor to initialize the grid and coin_location
        # variables:
        super(BlindHeuristic, self).__init__(tower_state)

    def __call__(self, state):
        # The actual heuristic computation. The blind heuristic will simply
        # always return 0.
        return 0
    


def CallEuclidean(n):
    '''
    This function will exeucte the code for Heuristic 2.
    
    '''
    t = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument("n", help="Number of discs", nargs="?", default=3)
    parser.add_argument("--heuristic", help="Which heuristic to use.The blind heuristic will turn A* into simple Breadth-First Search.", choices=['Eucldistancefromlastpeg'], default="Eucldistancefromlastpeg")
    args = parser.parse_args()
    
    n = n
    #Object for tower world with 4 pegs and n discs
    tower_state = Tower(n, 4)
    
    
    h = globals()["%sHeuristic" % args.heuristic](tower_state)
    
    t = time.time()
    result = astar_search(tower_state, h)
    
    
    print("States expanded: %d" % result.expansions)
    print("States visited:  %d" % result.visited)
    print("Total time:      %.3fs" % (time.time() - t))
    return time.time() - t,result.visited,result.expansions,(len(result.plan) - 1)

def CallManhatten(n):
    '''
    This function will exeucte the code for Heuristic 1.
    
    '''
    t = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument("n", help="Number of discs", nargs="?", default=3)
    parser.add_argument("--heuristic", help="Which heuristic to use.The blind heuristic will turn A* into simple Breadth-First Search.", choices=['Manhatdistancefromlastpeg'], default="Manhatdistancefromlastpeg")
    args = parser.parse_args()
    
    n = n
    #Object for tower world with 4 pegs and n discs
    tower_state = Tower(n, 4)
    
    
    h = globals()["%sHeuristic" % args.heuristic](tower_state)
    
    t = time.time()
    result = astar_search(tower_state, h)
    
    
    print("States expanded: %d" % result.expansions)
    print("States visited:  %d" % result.visited)
    print("Total time:      %.3fs" % (time.time() - t))
    return time.time() - t,result.visited,result.expansions,(len(result.plan) - 1)

def CallBlind(n):
    '''
    This function will exeucte the code for BFS.
    
    '''
    t = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument("n", help="Number of discs", nargs="?", default=3)
    parser.add_argument("--heuristic", help="Which heuristic to use.The blind heuristic will turn A* into simple Breadth-First Search.", choices=['Blind'], default="Blind")
    args = parser.parse_args()
    
    n = n
    #Object for tower world with 4 pegs and n discs
    tower_state = Tower(n, 4)
    
    
    h = globals()["%sHeuristic" % args.heuristic](tower_state)
    
    t = time.time()
    result = astar_search(tower_state, h)
    
    
    print("States expanded: %d" % result.expansions)
    print("States visited:  %d" % result.visited)
    print("Total time:      %.3fs" % (time.time() - t))
    
    return time.time() - t,result.visited,result.expansions,(len(result.plan) - 1)

def plot_graph(n,t,title):
    '''logic to plot the graph for  number of disc's vs runtime.
    '''
    #n=[2,3,4]
    n=[2,3,4,5,6,7,8,9,10]
    plt.plot(n,t)
    plt.grid()
    plt.title(title)
    plt.xlabel('no. of disks')   
    plt.ylabel('Time')
    plt.show()
        
    
    
    
    
if __name__ == "__main__":
    
    n = 3
    CallEuclidean(n)
    CallManhatten(n)  
    CallBlind(n)
    
    '''
    To see the enite results and plot run the below snippet.
    '''
    
    '''
    t1=[]
    t2=[]
    t3=[]
    Eucl_visited=[]
    Blind_visited=[]
    Manh_visited=[]
    Eucl_expanded=[]
    Blind_expanded=[]
    Manh_expanded=[]
    Eucl_plan=[]
    Blind_plan=[]
    Manh_plan=[]
    n = range(2,11) # number of discs
    for i in n:
        t1.append(CallEuclidean(i)[0])
        t2.append(CallBlind(i)[0])
        t3.append(CallManhatten(i)[0])
        Eucl_visited.append(CallEuclidean(i)[1])
        Manh_visited.append(CallManhatten(i)[1])
        Blind_visited.append(CallBlind(i)[1])
        Eucl_expanded.append(CallEuclidean(i)[2])
        Manh_expanded.append(CallManhatten(i)[2])
        Blind_expanded.append(CallBlind(i)[2])
        Eucl_plan.append(CallEuclidean(i)[3])
        Blind_plan.append(CallBlind(i)[3])
        Manh_plan.append(CallManhatten(i)[3])
        
        
    b=[]
    k=[0,1,2,3,4,5,6,7,8]
    for i in k:
    t1.append(a[i][0])
    

    
    print('Euclidean States visited are',Eucl_visited)
    print('Euclidiean Expanded are ',Eucl_expanded)
    print('Manhatten States visited are',Manh_visited)
    print('Manhatten Expanded are ',Manh_expanded)
    print('Blind States visited are',Blind_visited)
    print('Blind Expanded are ',Blind_expanded)
    
    print('time taken by Euclidiean is',t1)
    print('time taken by Manhatten is',t2)
    print('time taken by Blind is',t3)
    plot_graph(n, t1,'Euclidean')
    plot_graph(n, t2,'Manhatten')
    plot_graph(n, t3,'Blind')
    print("Euclidean Plan length:     ", Eucl_plan)
    print("Blind Plan length:     " , Blind_plan)
    print("Manhatten Plan length:     " , Manh_plan)
    '''
    
    