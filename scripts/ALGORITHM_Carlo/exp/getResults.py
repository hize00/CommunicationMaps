import os

env_name = ['bwopen0' , 'offices1']
ranges = [100, 250, 500, 1000]
results_file = ['resultsT_' , 'resultsD_' ]

def parseFile(file):
	#check if file exists
	if os.path.isfile(file) != True:
		print str(file) + " : FILE doesnt exist."
		return 
	#PARSING RESULTS FILE
	else:
		print file
		#with open(file) as f1:
		

#---------------------------
for e in env_name:
	for r in ranges:
		path = 'data_' + str(e) + "_" + str(r)
		for rf in results_file:
			file_to_open = path + "/" + str(rf) + str(e) + "_" + str(r) + ".txt"
			parseFile(file_to_open)

result = {}
result["offices"] = {}
result["offices"][100] = {}
result[["offices"][100].append(15)
result["offices"][100].append(23)

print result
print result["offices"]
print result["offices"][100]