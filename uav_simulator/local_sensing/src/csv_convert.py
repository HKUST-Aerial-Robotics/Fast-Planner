import csv
file_location = '/home/wang/bag/banchmark/EuRoC/ViconRoom101/state_groundtruth_estimate0/data.csv'
with open('/home/wang/bag/banchmark/EuRoC/ViconRoom101/state_groundtruth_estimate0/data.txt', 'w') as txt_f:
	with open(file_location) as f:
	    f_csv = csv.reader(f)
	    headers = next(f_csv)
	    for row in f_csv:
	    	txt_f.write('%lf\n'% (float(row[0]) / 1000000000.0) )
	    	txt_f.write('%lf\n'% (float(row[1])) )
	    	txt_f.write('%lf\n'% (float(row[2])) )
	    	txt_f.write('%lf\n'% (float(row[3])) )
	    	txt_f.write('%lf\n'% (float(row[4])) )
	    	txt_f.write('%lf\n'% (float(row[5])) )
	    	txt_f.write('%lf\n'% (float(row[6])) )
	    	txt_f.write('%lf\n'% (float(row[7])) )