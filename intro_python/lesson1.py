'''
lesson 1
   keyboard input, screen output
   converting strings to int, float
'''

''' 
input function reads from the keyboard
returns a string type
assignment operator - stores data returned in a variable
'''

stringMessage1 = input('Enter a word: ')
print("stringMessage1: " + stringMessage1)
print("data type of stringMessage1: " + str(type(stringMessage1)))

stringMessage2 = input("\nenter several words: ")
print("stringMessage2: " + stringMessage2)
print("data type of stringMessage2: " + str(type(stringMessage2)))

# convert the string to an integer
# if you type a non-numeric value, the conversion to int will fail
integerValue1 = int(input("\nenter integer value: "))

# when using the print function, the + operator appends to
# a string. The integer is converted to a string for printing
print("integerValue1: " + str(integerValue1))
print("data type of integerValue1: " + str(type(integerValue1)))

#convert the string to a float
floatValue1 = float(input("\nenter float value: "))

# when using the print function, the + operator appends to
# a string. The float is converted to a string for printing
print("integerValue1: " + str(floatValue1))
print("data type of floatValue1: " + str(type(floatValue1)))
