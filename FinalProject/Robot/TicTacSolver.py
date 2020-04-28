import rospy
import roslib.srvs
import argparse
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from srv._TicTacSolver import TicTacSolver
import string
import copy as cp

g_Namespace = ""


#arg is string representation of game grid 


# The game grid below is encoded into the following string.
# Positions are marked with either p1, p2, or _ 
# Game state string is prefixed by the current player
#
#   _11_|_12_|_13_
#   _21_|_22_|_23_
#   _31_|_32_|_33_
#
# p1:11,12,13|21,22,23|31,32,33
#
#
#
# Here is an actual game state representation
#   __|p1|p2
#   p2|p1|__
#   __|__|__
#
# p1:_,p1,p2|p2,p1,_|_,_,_
#
#
#
# The calculator should determine the best play position and return the string representation of the cell
# In the above example, the solver should return 32, which will win the game for player 1

class State:
    def __init__(self, moves):
        self.to_move = 'p1'
        self.utility = 0
        self.board = {}
        self.moves = cp.copy(moves)

        
class TicTacToe:
    
    def __init__(self, nrow=3, ncol=3, nwin=3, nexp=0):
        self.nrow = nrow
        self.ncol = ncol
        self.nwin = nwin
#        moves = # insert your general list of nrow x ncol moves here
        moves = [(row, col) for row in range(1, nrow + 1) for col in range(1, ncol + 1)]
        self.state = State(moves)
        self.nexp = nexp
        
    def returnState():
        return self.State

    def result(self, move, state):
        '''
        What is the hypothetical result of move `move` in state `state` ?
        move  = (row, col) tuple where player will put their mark (X or O)
        state = a `State` object, to represent whose turn it is and form
                the basis for generating a **hypothetical** updated state
                that will result from making the given `move`
        '''

        # your code goes here
        
        # Solution:
        # Don't do anything if the move isn't a legal one
        if move not in state.moves:
            return state
        # Return a copy of the updated state:
        #   compute utility, update the board, remove the move, update whose turn
        new_state = cp.deepcopy(state)
        new_state.utility = self.compute_utility(move, state)
        new_state.board[move] = state.to_move
        new_state.moves.remove(move)
        new_state.to_move = ('p2' if state.to_move == 'p1' else 'p1')
        return new_state
    
    def compute_utility(self, move, state):
        '''
        What is the utility of making move `move` in state `state`?
        If 'X' wins with this move, return 1;
        if 'O' wins return -1;
        else return 0.
        '''        

        # your code goes here

        # Solution:
        row, col = move
        player = state.to_move
        
        # create a hypothetical copy of the board, with 'move' executed
        board = cp.deepcopy(state.board)
        board[move] = player

        # what are all the ways 'player' could with with 'move'?
        
        # check for row-wise win
        in_a_row = 0
        for c in range(1,self.ncol+1):
            in_a_row += board.get((row,c))==player

        # check for column-wise win
        in_a_col = 0
        for r in range(1,self.nrow+1):
            in_a_col += board.get((r,col))==player

        # check for NW->SE diagonal win
        in_a_diag1 = 0
        for r in range(row,0,-1):
            in_a_diag1 += board.get((r,col-(row-r)))==player
        for r in range(row+1,self.nrow+1):
            in_a_diag1 += board.get((r,col-(row-r)))==player

        # check for SW->NE diagonal win
        in_a_diag2 = 0
        for r in range(row,0,-1):
            in_a_diag2 += board.get((r,col+(row-r)))==player
        for r in range(row+1,self.nrow+1):
            in_a_diag2 += board.get((r,col+(row-r)))==player
        
        if self.nwin in [in_a_row, in_a_col, in_a_diag1, in_a_diag2]:
            return 1 if player=='p1' else -1
        else:
            return 0
        

    def game_over(self, state):
        '''game is over if someone has won (utility!=0) or there
        are no more moves left'''

        # your code goes here
        
        # Solution:
        return state.utility!=0 or len(state.moves)==0    

    
    def utility(self, state, player):
        '''Return the value to player; 1 for win, -1 for loss, 0 otherwise.'''

        # your code goes here
        
        # Solution:
        return state.utility if player=='p1' else -state.utility        
        
        
    def display(self):
        for row in range(1, self.nrow+1):
            for col in range(1, self.ncol+1):
                print(self.state.board.get((row, col), '.'), end=' ')
            print()
        
    def play_game(self, player1, player2):
        '''Play a game of tic-tac-toe!'''

        # your code goes here

        # Solution:
        turn_limit = self.nrow*self.ncol  # limit in case of buggy code
        turn = 0
        while turn<=turn_limit:
            for player in [player1, player2]:
                turn += 1
                move = player(self)
                self.state = self.result(move, self.state)
                if self.game_over(self.state):
                    #self.display()
                    return self.state.utility                  

def alphabeta_search(game):
    '''search game approach to find best action, using alpha-beta pruning:
    alpha = best (highest) move found so far for Max
    beta  = best (lowest) move found so far for Min'''

    player = game.state.to_move

    # Functions used by alphabeta
    def max_value(state, alpha, beta):
        if game.game_over(state):
            return game.utility(state, player)
        value = -float('inf')
        for a in state.moves:
            
            value = max(value, min_value(game.result(a, state), alpha, beta))
            if value >= beta:
                return value
            alpha = max(alpha, value)
        return value

    def min_value(state, alpha, beta):
        if game.game_over(state):
            return game.utility(state, player)
        value = float('inf')
        for a in state.moves:
            
            value = min(value, max_value(game.result(a, state), alpha, beta))
            if value <= alpha:
                return value
            beta = min(beta, value)
        return value

    # Body of alphabeta_cutoff_search:
    best_score = -float('inf')
    beta = float('inf')
    best_action = None
    for a in game.state.moves:
        
        value = min_value(game.result(a, game.state), best_score, beta)
        if value > best_score:
            best_score = value
            best_action = a
    return best_action                    


def calculateBestMove(arg):
    #get python string from arg data
    print(arg)
    gameStateRaw = arg.str
    # Initialize game board
    ttt = TicTacToe()
    gameStateColonSplit = gameStateRaw.split(":")
    player = gameStateColonSplit[0]
    gameStateStr = gameStateColonSplit[1]
    gameStateLines = gameStateStr.split("|")
    for i in range(len(gameStateLines)):
      line = gameStateLines[i]
      cellArr = line.split(",")
      for j in range(len(cellArr)):
        cell = cellArr[j]
        if (cell != "_"):
          position = (i + 1, j + 1)
          ttt.state.to_move = cell
          # This indicated the game has already been won
          if (ttt.compute_utility(position, ttt.state) != 0):
            return ""
          ttt.state.board[position] = cell
          ttt.state.moves.remove(position)
    
    ttt.state.to_move = player
    # Print state
    ttt.display()
    print(f"to_move = {ttt.state.to_move}")
    print(f"utility = {ttt.state.utility}")
    print(f"board = {ttt.state.board}")
    print(f"moves = {ttt.state.moves}")
    move = alphabeta_search(ttt)
    ttt.state.board[move] = player
    ttt.display()
    print(move)
    
    return "stub"




def init(args):
    global g_Namespace

    g_Namespace = args.namespace
    rospy.init_node("%s_GameSolver" % g_Namespace)
    s = rospy.Service('%s_CalculateBestMove' % g_Namespace, TicTacSolver, calculateBestMove)
    rospy.spin()


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="TicTac_GameSolver")
  parser.add_argument('-n','--namespace', type=str, nargs='?', default='TicTac', help='Prepended string for all topics')
  args = parser.parse_args()

  init(args)