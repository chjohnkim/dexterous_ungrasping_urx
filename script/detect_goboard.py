#!/usr/bin/env python
import yaml
import numpy as np

def board2world_coordinate(board_coordinate):
    '''Input desired go coordinate and return 3D world coordinate. 

    Parameters:
        board_coordinate (string): example is 'A1' to 'S19' on a 19x19 go board.
    Returns:
        world_coordinate (list): [x, y, z] coordinates w.r.t. world frame.
    '''

    with open("/home/john/Desktop/dexterous_ungrasping_urx/config/go_board_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    board_dimension = config['board_dimension'] # standard go board dimension is 19x19
    board_height = config['board_height'] 
    corner_BL = config['corner_BL'] 
    corner_BR = config['corner_BR']
    corner_TL = config['corner_TL']
    corner_TR = config['corner_TR']
    
    corner_BL[2] = board_height
    corner_BR[2] = board_height
    corner_TL[2] = board_height
    corner_TR[2] = board_height

    letter = ord(board_coordinate[0].upper())-64
    if letter > 8:
        letter = letter - 1
    number = int(board_coordinate[1:])
    
    # Calculate coordinates of bottom row, starting from row 1
    bottom_row_increment = np.divide(np.subtract(corner_BR, corner_BL), board_dimension-1)
    bottom_row_coordinates = []
    for i in range(board_dimension):
        coordinate = np.add(corner_BL, np.multiply(bottom_row_increment, i))
        bottom_row_coordinates.append(coordinate)

    # Calculate coordinates of top row
    top_row_increment = np.divide(np.subtract(corner_TR, corner_TL), board_dimension-1)
    top_row_coordinates = []
    for i in range(board_dimension):
        coordinate = np.add(corner_TL, np.multiply(top_row_increment, i))
        top_row_coordinates.append(coordinate)

    go_board_coordinates = []
    for i in range(board_dimension):
        column_increment = np.divide(np.subtract(top_row_coordinates[i], bottom_row_coordinates[i]), board_dimension-1)
        column_coordinates = []
        for j in range(board_dimension):
            coordinate = np.add(bottom_row_coordinates[i], np.multiply(column_increment, j))
            column_coordinates.append(coordinate)
        go_board_coordinates.append(column_coordinates)
    
    return go_board_coordinates[letter-1][number-1]


if __name__ == '__main__':
    print board2world_coordinate('i7')
