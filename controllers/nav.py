import os.path


class cell:
    def __init__(self):
        self.visited = False
        self.wall = [False, False, False, False]  # Up, Right, Down, Left
        self.victim = False
        self.checkpoint = False

    def blackout(self):
        self.wall = [True, True, True, True]  # All walls up
        self.visited = True


class Nav:
    def __init__(self):
        print("Initialized AI!")
        # 2D array
        self.initialPosition = (10, 10)
        self.rows, self.cols = (20, 20)
        #self.field = [[cell()]*self.cols]*self.rows
        self.field = [[cell() for j in range(self.cols)]
                      for i in range(self.rows)]
        self.startPosition = self.initialPosition
        # row, column. Robot's current position on the field
        self.location = self.startPosition
        self.direction = 0  # The current direction of the robot
        self.previousLocation = (-1, -1)
        if os.path.isfile('wall.txt'):
            self.filePtr = open("wall.txt", 'r+')
        else:  # If wall.txt does not exist
            print("Creating wall.txt!")
            self.filePtr = open('wall.txt', 'w+')
            self.markVisited(writeToFile=True)
        if self.filePtr:
            print("Wall.txt is opened successfully")
        else:
            print("Wall.txt did not open successfully. . .")
            raise SystemExit

        # Read in whole file, and then turn string into list of words
        data = self.filePtr.read().split()
        while len(data) > 0:
            row = int(data.pop(0))
            col = int(data.pop(0))
            typeOfData = data.pop(0)
            print("Read in:", row, col, typeOfData)
            if typeOfData == 'V':
                print("Read visited!")
                self.field[row][col].visited = True
            elif typeOfData == "VICTIM":
                print("Read victim!")
                self.field[row][col].victim = True
            elif typeOfData == "CHECKPOINT":
                print("Read checkpoint!")
                self.field[row][col].checkpoint = True
                # Update starting position to last found checkpoint
                self.startPosition = (row, col)
            else:  # Data is giving a wall direction
                self.markWall(typeOfData, (row, col), writeToFile=False)
        # mark victim at current location to make sure robot knows current tile is visited

    # Converts a direction in str to int format or int to str format
    def convertDirection(self, direction):
        if isinstance(direction, str):
            if direction == 'NORTH':
                return 0
            if direction == 'EAST':
                return 1
            if direction == 'SOUTH':
                return 2
            if direction == 'WEST':
                return 3
        elif isinstance(direction, int):
            if direction == 0:
                return 'NORTH'
            if direction == 1:
                return 'EAST'
            if direction == 2:
                return 'SOUTH'
            if direction == 3:
                return 'WEST'
        else:
            print("Direction Conversion Error!")
            raise SystemExit

    # Converts a command in str to int format or int to str format
    def convertCommand(self, command):
        if isinstance(command, str):
            if command == 'FORWARD':  # Move forward
                return 0
            if command == 'RIGHT':  # Turn right 90 deg, then move forward
                return 1
            if command == 'BACKWARD':  # Turn 180 deg, then move forward
                return 2
            # Turn left 90 deg (-90 deg), then move forward
            if command == 'LEFT':
                return 3
        elif isinstance(command, int):
            if command == 0:
                return 'FORWARD'
            if command == 1:
                return 'RIGHT'
            if command == 2:
                return 'BACKWARD'
            if command == 3:
                return 'LEFT'
        else:
            print("Command Conversion Error!")
            raise SystemExit

    # This function returns the correct relative command for the robot to traverse. Call this funtion BEFORE you update self.location during calculate()!
    # @return Returns the needed command given circumstances and returns the new direction of the robot after executing this command
    def determineCommand(self, currentDirection, newPosition, oldPosition=None):
        if oldPosition is None:  # Defualt oldPosition to current location
            oldPosition = self.location

        # 1. Calculate the new needed direction
        adjustRow = newPosition[0] - oldPosition[0]
        adjustCol = newPosition[1] - oldPosition[1]
        if adjustRow == 1:  # Moving up
            newDirection = 0
        elif adjustCol == 1:  # Moving right
            newDirection = 1
        elif adjustRow == -1:  # Moving down
            newDirection = 2
        else:  # Moving left
            newDirection = 3

        # 2. Calculate what command to adjust the current direction into the new direction
        adjustDirection = ((newDirection-currentDirection) + 4) % 4
        if adjustDirection == 0:
            return 'FORWARD', newDirection
        elif adjustDirection == 1:
            return 'RIGHT', newDirection
        elif adjustDirection == 2:
            return 'BACKWARD', newDirection
        else:
            return 'LEFT', newDirection

    # This function takes the direction of the wall, enters it at a specified location [or the current location], and optionally writes the wall data to a file.
    # str, [tuple], [boolean]
    def markWall(self, direction, loc=None, writeToFile=True):
        if loc is None:  # Defualt loc to current location
            loc = self.location
        if isinstance(direction, str):  # Convert direction to integer
            direction = direction.upper()  # uppercase everything to match formatting
            direction = self.convertDirection(direction)

        self.field[loc[0]][loc[1]].wall[direction] = True  # Mark Wall

        direction = self.convertDirection(direction)  # Convert direction to string
        if writeToFile:
            # Row, Col, Direction
            self.filePtr.write(str(loc[0]) + ' ' +
                               str(loc[1]) + ' ' + direction + '\n')
            print("Entered & wrote wall: ", direction)
        else:
            print("Entered wall: ", direction)

    def markCheckpoint(self, loc=None):
        if loc is None:  # Defualt loc to current location
            loc = self.location
        print("Wrote Checkpoint to File!")
        # Row, Col, Direction
        self.filePtr.write(str(loc[0]) + ' ' + str(loc[1]) + ' ' + 'CHECKPOINT' + '\n')
        self.field[loc[0]][loc[1]].checkpoint = True  # Mark Victim

    def markVisited(self, loc=None, writeToFile=True):
        if loc is None:  # Defualt loc to current location
            loc = self.location
        print("Wrote Visited to File!")
        self.field[loc[0]][loc[1]].visited = True  # Mark Victim
        if writeToFile:
            # Row, Col, Direction
            self.filePtr.write(str(loc[0]) + ' ' + str(loc[1]) + ' ' + 'V' + '\n')

    # Marks seen victim at current location
    def markVictim(self, loc=None):
        if loc is None:  # Defualt loc to current location
            loc = self.location
        print("Wrote Victim to File!")
        # Row, Col, Direction
        self.filePtr.write(str(loc[0]) + ' ' +
                           str(loc[1]) + ' ' + 'VICTIM' + '\n')
        self.field[loc[0]][loc[1]].victim = True  # Mark Victim

    # Function if want to move up
    # if backtrack = true, only returns if you can or can't move up. If you can, then also returns coordinates
    # @return: if True, then moved will be successfull. Will then also return new coordinates. if False, then move will not be allowed.
    def canMove(self, direction, loc=None, backtrack=False):
        if loc is None:  # Defualt loc to current location
            loc = self.location
        if isinstance(direction, str):  # Convert direction to integer
            direction = direction.upper()  # uppercase everything to match formatting
            direction = self.convertDirection(direction)

        # Convert direction to adjust row & adjust col
        adjustRow = 0
        adjustCol = 0
        if direction == 0:  # Moving up
            adjustRow = 1
        elif direction == 1:  # Moving right
            adjustCol = 1
        elif direction == 2:  # Moving down
            adjustRow = -1
        else:  # Moving left
            adjustCol = -1

        newLoc = (loc[0] + adjustRow, loc[1] + adjustCol)  # Row, Col
        # If coordinates are out of bounds
        if newLoc[0] < 0 or newLoc[0] >= self.rows or newLoc[1] < 0 or newLoc[1] >= self.cols:
            return False, None
        # If there is a wall in the direction trying to go
        if self.field[loc[0]][loc[1]].wall[direction]:
            return False, None
        # If robot isn't backtracking but the new tile is already visited
        if (not backtrack) and self.field[newLoc[0]][newLoc[1]].visited:
            return False, None
        return True, newLoc

    def calculate(self):
        newDirection, newLocation, foundMove, commands = 1, (-1, -1), False, list()  # Default values
        for direction in range(0, 4):
            moveIsPossible, newLocation = self.canMove(
                direction, backtrack=False)
            if moveIsPossible:
                command, newDirection = self.determineCommand( self.direction, newLocation)
                commands.append(command)
                foundMove = True
                break

        if not foundMove:  # Need to BFS
            commands, newLocation, newDirection = self.backtrackBFS()

        self.direction = newDirection  # Update Direction
        self.previousLocation = self.location  # Update previous location
        self.location = newLocation  # Update location to new location
        self.markVisited(newLocation, writeToFile=True)  # Mark new location as visited

        # if commands == None: # Then backtrack home
        return commands

    # Finds the path to the nearest unvisited tile via BFS
    # @return Returns the set of commands to reach the new location, the new location's coordinates, and the new direction. Does NOT update location & direction for you
    def backtrackBFS(self):
        print("BFS Initiated!")
        # visited = [[False]*self.cols]*self.rows  # Visited array
        visited = [[False for j in range(self.cols)] for i in range(self.rows)]
        # prevCell = [(-1, -1)*self.cols]*self.rows  # Previous cell array
        prevCell = [[[-1, -1] for j in range(self.cols)] for i in range(self.rows)]
        queue = list()
        queue.append(self.location)  # first add current location
        targetLocation = None
        while len(queue) > 0 and targetLocation == None:
            currentCell = queue.pop(0)
            visited[currentCell[0]][currentCell[1]] = True
            for direction in range(0, 4):
                moveIsPossible, newCell = self.canMove(direction, currentCell, backtrack=True)
                if moveIsPossible:
                    # if new cell is unvisited and possible to access
                    if self.field[newCell[0]][newCell[1]].visited == False:
                        prevCell[newCell[0]][newCell[1]] = (currentCell[0], currentCell[1])
                        targetLocation = newCell
                        break
                    # new cell is not in the queue and new cell has not been visited yet
                    elif (not newCell in queue) and visited[newCell[0]][newCell[1]] == False:
                        prevCell[newCell[0]][newCell[1]] = (currentCell[0], currentCell[1])
                        queue.append(newCell)
                    else:
                        pass

        if targetLocation == None:  # No more unvisited tiles
            return None  # return none to indicate go back home

        # Now it's time to backtrack each location! One by One!
        locations = [targetLocation]
        newPosition = list(targetLocation)
        while newPosition != self.location:
            newPosition = prevCell[newPosition[0]][newPosition[1]]
            locations.append(newPosition)
        locations.reverse()

        # With each tile's coordinates on the path, we can now calculate the commands
        commands = list()
        currentPosition = locations[0]
        currentDirection = self.direction
        i = 1
        while i < len(locations):
            newPosition = locations[i]
            command, newDirection = self.determineCommand(currentDirection, newPosition, oldPosition=currentPosition)
            commands.append(command)
            currentPosition = newPosition
            currentDirection = newDirection
            i += 1

        return commands, currentPosition, currentDirection
