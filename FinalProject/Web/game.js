// Initialize some global variables
let cells = ['11', '12', '13', '21', '22', '23', '31', '32', '33'];
let board = {};  // Board State
let playerSymbol = 'X';  // What symbol the human will play with
let simSymbol = 'O';
let toMove = 'X';  // Whos turn it is
let enabled = true;

// Initialize each cell to null
cells.forEach(function(cell, index) {
   board[cell] = '-';
});

// Do some set-up onload
window.onload = function() {
    initializeROS();

    let windowHeight = window.innerHeight;

    // Make left and right section go full-height
    document.getElementById('game-section').style.height = windowHeight + "px";
    document.getElementById('info-section').style.height = windowHeight + "px";

    // Center game board in its section
    let gameSectionWidth = Math.round(document.getElementById('game-section').offsetWidth);
    let gameSectionHeight = Math.round(document.getElementById('game-section').offsetHeight);
    let gameBoardWidth = Math.round(document.getElementById('11').offsetWidth * 3);
    let gameBoardHeight = Math.round(document.getElementById('11').offsetHeight * 3);
    let sidebarWidth = Math.round(document.getElementById('info-section').offsetWidth);

    // Calculate exact horizontal center, 1/3rd of the vertical center
    let horizontalPadding = (gameSectionWidth - gameBoardWidth) / 2;
    let verticalPadding = (gameSectionHeight - gameBoardHeight) * 0.333;

    // + 30 = 3 margins of 10px each
    let titlePadding = gameBoardWidth + 30;

    // Apply
    document.getElementById('board').style.paddingLeft = horizontalPadding + "px";
    //document.getElementById('board').style.paddingRight = horizontalMargin + "px";
    document.getElementById('board').style.paddingTop = verticalPadding + "px";

    // Game title is centered inside by parent element CSS, so just have to make the width match the game board width
    document.getElementById('gameTitle').style.width = titlePadding + "px";

    // Adjust sidebar
    document.getElementById('bottomSection').style.width = sidebarWidth + "px";
    document.getElementById('resetButton').style.width = (sidebarWidth - 32) + "px";

    // Render initial board
    renderBoard();
};

// Input: board is a dictionary with keys = unique id's of each cell on the board, val = who the cell belongs to
// Updates the HTML to represent the current board state
function renderBoard() {
    for (const [key, val] of Object.entries(board)) {
        document.getElementById(key).innerHTML = '<br />' + val.toString();
    }

    let winner = checkWinner();

    // If no winner, update text to display whos turn it is
    if (winner === 0) {
        if (toMove == playerSymbol) {
            document.getElementById('gameInfo').innerText = 'Current Turn: Player';
        } else if (toMove == simSymbol) {
            document.getElementById('gameInfo').innerText = 'Current Turn: Robot';
        }
    }
    else {
        if (winner === 1) {
            document.getElementById('gameInfo').innerText = 'Winner: Player';
        }
        else if (winner == -1) {
            document.getElementById('gameInfo').innerText = 'Winner: Robot';
        }
    }
}

// Called from checkWinner() if we have a winner.
// winner = player who won, winningCells = unique HTML id's of winning cells
// Goes through game board and highlights which cells won a game of tic-tac-toe
function highlightWinningCells(winningCells, winner) {
    let color = 'green';
    if (winner !== 1) {
        color = 'red';
    }

    winningCells.forEach(function(val, index) {
        document.getElementById(val).style.color = color;
    })
}

// Converts a (row, col) pair to our shorthand cell notation
// I.e. passing in (3, 2) returns 32
function rowColToCell(row, col) {
    return row.toString().toLowerCase().trim() + col.toString().toLowerCase().trim();
}

// Takes in a (row, col) pair and returns the value of the game board at that (row, col) (i.e. X, O, or -)
function getCellValue(row, col) {
    let cell = rowColToCell(row, col);
    if (!cells.includes(cell)) {
        return null;
    }
    else {
        return board[cell].toString().toUpperCase().trim();
    }
}

// This is a helper function for checkWinner() which takes in our two sets and sees if there is a winner + if there is,
// which player won
function checkWinnerHelper(symbols, cells) {
    // Check if there's only one symbol in our set. If there is, there is 3 in a row!
    if (symbols.size === 1) {
        // Check if the only symbol is the players...
        if (symbols.has(playerSymbol)) {
            highlightWinningCells(cells, 1);
            enabled = false;
            return 1;
        }
        // Check if the only symbol is the robots
        else if (symbols.has(simSymbol)) {
            highlightWinningCells(cells, -1);
            enabled = false;
            return -1;
        }
    }
    return 0;
}


// This function looks at our current board state (held in dictionary board) and determines if we have a winner or not
function checkWinner() {
    let winner = 0;  // 0 = no winner, 1 = player win, -1 = robot win
    var results = new Set();  // Will hold the values of cells we are checking (i.e. will hold X or O)
    var winningCells = new Set();  // Will hold the cell locations we are checking

    /* STEP 1: CHECK FOR WINNERS ROW-WISE FIRST */
    // Iterate through each row
    for (let row = 1; row < 4; row++) {
        // In-between each row, clear our sets
        results.clear();
        winningCells.clear();

        // Iterate through each col in each row, add the values + locations of cells to our sets
        for (let col = 1; col < 4; col++) {
            results.add(getCellValue(row, col));
            winningCells.add(rowColToCell(row, col));
        }

        // Check our sets for a winner, if one exists break out of function
        winner = checkWinnerHelper(results, winningCells);
        if (winner !== 0) {return winner}

    }

    /* STEP 2: CHECK FOR WINNERS COLUMN-WISE */
    // Iterate through each column
    for (let col = 1; col < 4; col++) {
        // In-between each column, clear our sets
        results.clear();
        winningCells.clear();

        // Iterate through each row in each column, add the values + locations of cells to our sets
        for (let row = 1; row < 4; row++) {
            results.add(getCellValue(row, col));
            winningCells.add(rowColToCell(row, col));
        }

        // Check our sets for a winner, if one exists break out of function
        winner = checkWinnerHelper(results, winningCells);
        if (winner !== 0) {return winner}
    }


    /* STEP 3: CHECK FOR WINNERS DIAGONAL */
    // Check top-left to bottom-right diagonal
    results.clear();
    winningCells.clear();
    let col = 1;
    for (let row = 1; row < 4; row++) {
        results.add(getCellValue(row, col));
        winningCells.add(rowColToCell(row, col));
        col += 1;
    }

    // Check our sets for a winner, if one exists break out of function
    winner = checkWinnerHelper(results, winningCells);
    if (winner !== 0) {return winner}

    // Check bottom-left to top-right diagonal
    results.clear();
    winningCells.clear();
    col = 1;
    for (let row = 3; row > 0; row--) {
        results.add(getCellValue(row, col));
        winningCells.add(rowColToCell(row, col));
        col += 1;
    }

    // Check our sets for a winner, if one exists break out of function
    winner = checkWinnerHelper(results, winningCells);
    if (winner !== 0) {return winner}

    // If no winner, return 0
    return 0;

}

// Reset game!
function resetGame() {
    board = {};  // Board State
    playerSymbol = 'X';  // What symbol the human will play with
    simSymbol = 'O';
    toMove = 'X';  // Whos turn it is
    enabled = true;

    // Initialize each cell to no move
    cells.forEach(function(cell, index) {
        board[cell] = '-';
    });

    // Render board
    renderBoard();

    // Reset all cell colors (since we highlight winning cells after a win)
    for (const [key, val] of Object.entries(board)) {
        document.getElementById(key).style.color = '#CFB87C';
    }
}


// Input: cell is what cell we wish to make a move in (id of cell, unique identifier)
// Input: player is the player (X or O) currently trying to make the move
// This function will verify that the player trying to make the move is allowed to make the move and then if so,
// will make the actual move and update the game state
function makeMove(cell, player) {
    // First check that cell exists on our board
    if (!cells.includes(cell)) {
        window.alert("Trying to make move in invalid cell " + cell.toString());
    }

    // If game has been won, we need to reset the board
    if (!enabled) {
        window.alert("Game is over, please reset game!");
        return null;
    }

    // Check if web player is trying to make move + if it is their turn to move
    // If so, update board + whos turn it is.
    if (player == 'web' && toMove == playerSymbol) {
        board[cell] = playerSymbol;
        //toMove = simSymbol;
        renderBoard();
    }
    else if (player == 'web' && toMove != playerSymbol) {
        window.alert('It is not your turn!');
        return null;
    }
    // Check if simulator/robot is trying to make move + if it is their turn to move
    // If so, update board + whos turn it is
    else if (player == 'sim' && toMove == simSymbol) {
        board[cell] = simSymbol;
        toMove = playerSymbol;
        renderBoard();
    }
}