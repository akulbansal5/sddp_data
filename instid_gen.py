import os
import csv

def append_instances_to_file(csv_file, stList, scenList, rowList, colList, rep):

    """
    stList: stages
    rowList: possible number of rows
    scenList: number of scenarios
    colList: number of columns
    rep: number of replications

    returns: appends to csv file instance ids which are then used for generating instances
    """

    with open(csv_file, 'r') as file:
        lines = file.readlines()

        if not lines:
            print("File is empty.")
            return


        last_line = lines[-1].strip()
        serial_number = last_line.split(',')[0]


        #id	inst	T	rows	cols	scens
        csv_rows = []
        id = int(serial_number) + 1
        
        for st in stList:
            for scens in scenList:
                for rows in rowList:
                    for cols in colList:
                        for r in range(rep):
                            csv_row = [id, 1, st, rows, cols, scens]
                            csv_rows.append(csv_row)
                            id = id + 1
    with open(csv_file, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(csv_rows)

    print(f"Total number of scenarios generated: {id - int(serial_number)}")

# Example usage:
dir_path    = os.path.dirname(os.path.realpath(__file__))
folder      = "/data/"
filename    = "instances.csv"
stList      = [3,4,5]
scenList    = [3,4,5]
rowList     = [10, 15]
colList     = [20, 30, 40]
reps        = 5
csv_file    = dir_path + folder + filename
append_instances_to_file(csv_file, stList, scenList, rowList, colList, reps)