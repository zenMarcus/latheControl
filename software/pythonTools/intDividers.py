counter = 26250 # int(pow(2,32))
print(counter)

for i in range(1,255):
    if (counter % (i) == 0):
        print(i,counter/i)