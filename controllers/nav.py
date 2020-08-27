class cell:
    def __init__(self):
        self.visited = False
        self.wall = [False, False, False, False] # Up, Right, Down, Left
        self.victim = False
        self.checkpoint = False
    
    def blackout(self):
        self.wall = [True, True, True, True] # All walls up
        self.visited = True
    

class Nav:
    startPosition = (10,10)
    def __init__(self):
        print("Initialized AI!")
        # 2D array 
        self.rows, self.cols = (20, 20) 
        self.field = [[cell()]*self.cols]*self.rows
        self.location = startPosition # row, column
        self.previous = (-1, -1)
        self.filePtr = open("wall.txt", 'r+')
        if self.filePtr:    
            print("file is opened successfully")
        data = self.filePtr.read().split() # Read in whole file, and then turn string into list of words
        while len(data) > 0:
            row = int(data.pop(0))
            col = int(data.pop(0))
            direction = data.pop(0)
            print("Read in:", row, col, direction)
            self.writeWall(direction, (row,col))
    
    def convertDirection(self, direction): # Converts a direction in str to int format or int to str format
        if isinstance(direction, str):
            if direction == 'UP':
                return 0
            if direction == 'RIGHT':
                return 1
            if direction == 'DOWN':
                return 2
            if direction == 'LEFT':
                return 3
        elif isinstance(direction, int):
            if direction == 0:
                return 'UP'
            if direction == 1:
                return 'RIGHT'
            if direction == 2:
                return 'DOWN'
            if direction == 3:
                return 'LEFT'
        else:
            print("Direction Conversion Error!")
            raise SystemExit
            quit()

    # This function takes the direction of the wall, enters it at a specified location [or the current location], and optionally writes the wall data to a file.
    def writeWall(self, direction, loc=None, writeToFile=False): # str, [tuple], [boolean]
        if loc is None: # Defualt loc to current location
            loc = self.location
        if isinstance(direction, str): # Convert direction to integer
            direction = direction.upper() # uppercase everything to match formatting
            direction = self.convertDirection(direction)
        
        self.field[loc[0]][loc[1]].wall[direction] = True # Mark Wall

        direction = self.convertDirection(direction) # Convert direction to string
        if writeToFile:
            self.filePtr.write(str(loc[0])+ ' ' + str(loc[1]) + ' ' + direction + '\n') # Row, Col, Direction
            print("Entered & wrote wall: ", direction)
        else:
            print("Entered wall: ", direction)

    # Function if want to move up
    # @return: if True, then successfully moved
    def moveUp(self, backtrack=False): #[boolean] if robot is backtracking (checking if tile is visited or not)
        loc = self.location
        if self.field[loc[0]][loc[1]].up:
            return False