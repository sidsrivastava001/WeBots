from nav import Nav

AI = Nav()
#print(AI.location)
#AI.markWall(3)
#AI.markVisited((4,4))
x = AI.calculate()

'''
 0,0 --> 1,0
UP
1. blackout() --> void
2. Undo action
3. calculate() --> will give you a list of commands
4. proceed

when you call black out
1. wall off dest., mark as visited
2. Revert back to previous position (0,0)
2. calculate(): calculate how to go to next closest unvisited square
3. Return a set of commands

'''