import heapq


class node(object):
    def __init__(self, x, y, occupied):
        self.priority = priority
        self.description = description
        self.x = x
        self.y = y
        self.parent = None
	self.H = heuristicFunction(self)
	self.G = previousG + 1
        self.f = somefunction(self.G, self.H)
        self.occupied = occupied
        
        return
    def __cmp__(self, other):
        return cmp(self.priority, other.priority)




class AStar(object):

    def __init__(self, mapwidth, mapheight, startpos, goal):
        self.opened = []
        
        heapq.heapify(self.opened)
        self.closed = set()
        self.cells = []
        self.width = mapwidth
        self.height = mapheight
        

    def init_grid(self, wallList)
        walls = wallList
        for x in range(self.width)
            for y in range(self.height)
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(node(x, y, reachable)
        self.start = self.get_cell(startpos.x, startpos.y)
        self.end = self.get_cell(goal.x, goal,y)



    def dankHeuristics(self, currentNode):
        return math.sqrt( (self.end.x - currentNode.x)^2 +  (self.end.y - currentNode.y)^2 ) 


    def get_cell(self, x, y):
        return self.cells[x*self.height+y] #return cell at location given, make sure this works. if it works, it's because of heapq


    def get_children(self, cell):
        cells = []
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x < self.width-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y < self.height-1:
            cells.append(self.get_cell(cell.x, cell.y+1))
        return cells


    def sortNode(nodeList):
        return sorted(nodeList, key=attrgetter('f')))
        




    def explore(mapnode, level=0):
    #take in node, then calculate h ang g for all children, place all in queue
        for i, child in mapnode.children:
            unexplored.place
    
    
