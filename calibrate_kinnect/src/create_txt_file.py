#!/usr/bin/python

number_files = 40
file = open("lists.txt","w") 

for n in range(number_files+1):
    if n <10:
        file.write("left-000"+str(n)+".png\n") 
        file.write("right-000"+str(n)+".png\n") 
    else:
        file.write("left-00"+str(n)+".png\n") 
        file.write("right-00"+str(n)+".png\n")         
 
file.close() 
