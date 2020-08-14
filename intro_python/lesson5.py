# lesson 5 - loops

count = 0

while (count < 3):
   print("while loop, count: ", count)
   count = count + 1


print("\nfor loop 1")
fruits = ['watermelon', 'grapes', 'apples']
for f in fruits:
   print(f)

print("\nfor loop range(len(fruits)) output")
for index in range(len(fruits)):
   print('Current fruit : ', fruits[index])

# python range function: range(start, stop, step)
print("\nfor loop range(4) output")

# i starts at 0, increases by 1, and stops after 3
for i in range(4):
   # end print statement with a comma instead of a newline
   print(i, end=', ')

print("\n\nfor loop range(3,8,2)")
# i starts at 3, stops after 7, increments by 2
for i in range(3, 8, 2):
   print(i, end=' ')

print('')