''' lesson 6 - functions

Several function definitions are listed below, immediately
followed by example code to call the function. This style is not
how programs are normally written. It was simply convenient to group
all the examples into one program.

We will see better style in future examples.

'''

import math       # sin, cos, pi 


# define function with no parameters
def hello_function():
   print('hello from hello_function')

# call function hello_function
hello_function()


# define function with one parameter
def print_argument(msg):
   print('function print_argument, msg: ', msg)

# call functin print_argument, pass 'cat in the hat' as an argument
# to function parameter msg
print_argument('cat in the hat')


# define function with multiple parameters
def print_names(fname, lname):
   print(fname + " " + lname)

# call function print_names. Pass two arguments to its parameters
# When calling the function, it expects two arguments because the 
# function was defined with two parameters. Try calling it with one or
# three parameters. You will get an error message.
print_names('Bart', 'Simpson')



# arbitrary arguments, *args
def some_function(*names):
   print('there are ' + str(len(names)) + ' names')
   for n in names:
      print(n)


# call some_function.
some_function('Bart', 'Homer', 'Marge', 'Lisa')




# keyword arguments
def average_of_three(a, b, c):
   avg = (a+b+c)/3
   print('average of a: {}, b: {}, c: {} is {}'.format(a,b,c,avg))


# call average_of_three, use key=value syntax
# does not require that we pass arguments in same order as parameters
average_of_three(c = 9, a = 7, b = 4)



# aribtrary keyword arguments, **kwargs
def print_knames(**person):
   print("Last name is " + person["lname"])

# call function, specify keyword arugments
print_knames(fname='Charlie', lname = 'Brown')



# default parameter value
def my_university(school = "CU Denver"):
   print("My university is " + school)

# call function, use default value
my_university()

# call function, pass value to parameter
my_university("Montana State")




# return one value
def scale_it(x):
   return 10 * x 

scaledVal = scale_it(3)

# return two values
def find_xy(radius, angle):
   x = radius * math.cos(angle)
   y = radius * math.sin(angle)
   return x, y 

# assigns x to a, b to y
a, b = find_xy(9, math.pi)



# pass statement allows function to be called
def emptyFunction():
   pass 

# call emptyFunction
emptyFunction()