# Introduction to Python Programming

"Python is an easy to learn, powerful programming language. It has efficient high-level data structures and a simple but effective approach to object-oriented programming. Python’s elegant syntax and dynamic typing, together with its interpreted nature, make it an ideal language for scripting and rapid application development in many areas on most platforms."[1](https://docs.python.org/3/tutorial/index.html)


The are many good python programming tutorials found on the Internet, as well as many good books to help you get started with python. In this module, we will cover just a few python basics needed to get started in this course.


## Books and Tutorial References

One of my favorite books: Python Crash Course, 2nd Edition: A Hands-On, Project-Based Introduction to Programming by Eric Mathes,  https://www.amazon.com/Python-Crash-Course-2nd-Edition/dp/1593279280



An internet search with keywords "python tutorial for beginners" will provide a myriad of choices. Tutorials point has good tutorials for many languages.

https://www.tutorialspoint.com/python/


There are many of YouTube channels providing tutorials. You may find one of the following channels helpful.

- Microsoft's Python for Beginners is a good series of short videos, including helping you configure Visual Studio Code. https://channel9.msdn.com/Series/Intro-to-Python-Development?WT.mc_id=python-c9-niner 

- techwithtim https://www.youtube.com/playlist?list=PLzMcBGfZo4-mFu00qxl0a67RhjjZj3jXm
- sentdex https://www.youtube.com/playlist?list=PLQVvvaa0QuDeAams7fkdcwOGBpGdHpXln
- corey shafer https://www.youtube.com/playlist?list=PL-osiE80TeTt2d9bfVyTiXJA-UTHn6WwU


The python language documentation also contains a set of tutorials and library documentation:

- The Python Tutorial https://docs.python.org/3/tutorial/index.html 
- The Python Standard Library  https://docs.python.org/3/library/index.html#library-index 



You will find there are references to python 2.x and 3.x. There are differences between these versions. Class examples will be written for python 3.x.

The examples in lessons 1 - 6 below are intended to introduce you to the python syntax and provide a quick start reference for the language.

<br><br>

## Lesson 1 - keyboard input, print output, converting strings to int, float

Lesson 1 demonstrates how to read input from the keyboard, convert the data from string to integer or float, and print the data. See lesson1.py.

### Comments

Everything to the right of the hash character # is a comment.

```
# This is a comment
print('this code will run')    # this will not run, is a comment
``` 

Multiline comments are wrapped inside triple quotes. Can use single or double quotes. 

```
"""
omment line 1
comment line 2
"""
```

Read this. "Writing Comments in Python"  https://realpython.com/python-comments-guide/ 


Library documentation for input, print:

https://docs.python.org/3/library/functions.html#input

https://docs.python.org/3/library/functions.html#print
<br><br>

## Lesson 2 - Formatting output using String modulo operator(%) 

The % operator can also be used for string formatting. It interprets the left argument much like a printf()-style format string to be applied to the right argument. In Python, there is no printf() function but the functionality of the ancient printf is contained in Python. To this purpose, the modulo operator % is overloaded by the string class to perform string formatting. Therefore, it is often called string modulo (or sometimes even called modulus) operator.

```
# print a with default number of decimal places, f is a placeholder for floating point type
a = 1.234
b = 5
print("a: %f" % (a))

# print a with 1 decimal place
print("a: %.2f" % (a))

# print b an integer value
print("b: %d" % (b))

# print two variables. The variables to be substituted into the placeholder are placed
# in () separated by commas
msg = 'hello'
print("msg: %s, b: %d" % (msg, b))
```

The newer method of formatting uses .format(), where {} become the placeholders, and the arguments are passed to the format function. See example below. https://docs.python.org/3/library/string.html


```
a = 1.234
b = 5
print("a: {}".format(a))
print("a: {}, b: {}".format(a,b))
```
<br><br>

## Lesson 3 - Arithmetic Operators

| Operator | Description | Example a = 10, b = 20 |
| --- | --- | --- |
| + | Addition 	Adds values on either side of the operator. | a + b = 30 |
| - |Subtraction 	Subtracts right hand operand from left hand operand. | a – b = -10 |
| * | Multiplication 	Multiplies values on either side of the operator | a * b = 200 |
| / | Division 	Divides left hand operand by right hand operand | b / a = 2 |
| % | Modulus 	Divides left hand operand by right hand operand and returns remainder | b % a = 0 |
| ** | Exponent 	Performs exponential (power) calculation on operators | a**b = 10 to the power 20 |


https://www.tutorialspoint.com/python/python_basic_operators.htm

See lesson3.py for example usage
<br><br>

## Lesson 4 - Comparison Operators, if statements

| Operator | Description | Example a = 10, b = 20 |
| --- | --- | --- |
| == | If the values of two operands are equal, then the condition becomes true. | (a == b) is not true. |
| != | If values of two operands are not equal, then condition becomes true. | (a != b) is true. |
| <> | If values of two operands are not equal, then condition becomes true. | (a <> b) is true. This is similar to != operator. |
| >  | If the value of left operand is greater than the value of right operand, then condition becomes true. | (a > b) is not true. |
| <  | If the value of left operand is less than the value of right operand, then condition becomes true. | (a < b) is true. |
| >= | If the value of left operand is greater than or equal to the value of right operand, then condition becomes true. | (a >= b) is not true. |
| <= | If the value of left operand is less than or equal to the value of right operand, then condition becomes true. | (a <= b) is true. |

<br>

**if statements**
- An if statement consists of a boolean expression followed by one or more statements.
- An if statement can be followed by an optional else statement, which executes when the boolean expression is FALSE.
- nested if statements
- - You can use one if or else if statement inside another if or else if statement(s).

In Python, all the statements indented by the same number of character spaces after a programming construct are considered to be part of a single block of code. Python uses indentation as its method of grouping statements.

See lesson4.py for example usage <br>
<br><br>

## Lesson 5 - loops

- while loop
- - Repeats a statement or group of statements while a given condition is TRUE. It tests the condition before executing the loop body.

```
while expression:
   statement(s)
```
<br>

- for loop
   - Executes a sequence of statements multiple times and abbreviates the code that manages the loop variable.

```
for iterating_var in sequence:
   statements(s)
```

If a sequence contains an expression list, it is evaluated first. Then, the first item in the sequence is assigned to the iterating variable iterating_var. Next, the statements block is executed. Each item in the list is assigned to iterating_var, and the statement(s) block is executed until the entire sequence is exhausted.
<br><br>

### range function: range(start, stop[, step])

range takes three arguments. Out of the three, two arguments are optional. I.e.,start and step are the optional arguments.

- A start argument is a starting number of the sequence. i.e., lower limit. By default, it starts with 0 if not specified.
- A stop argument is an upper limit. i.e., generate numbers up to this number. The range() doesn’t include this number in the result. The step is a difference between each number in the result. The default value of the step is 1 if not specified.

```
for var in range():
```

See lesson5.py for examples of all these loops.
<br><br>

## Lesson 6 - functions

A function is a block of code that only runs when it is called. Data can be passed to a function's arguments. A function can also return data.

### Creating a Function

A function is defined using the **def** keyword, followed by the function name, parentheses, optional parameters inside the parentheses and a colon. 

The following example defines a function that prints a message when called. It has no arguments and does not return any data. Statements inside the function body are indented with a tab.

```
# define the function
def hello_function():
   print('hello from hello_function')

# call function hello_function
hello_function()
```

<br>

The next example function has one parameter. It will print the data passed to the function parameter. The function does not return any data.<br>

```
def print_argument(msg):
   print('function print_argument, msg: ', msg)

# call functin print_argument, pass 'cat in the hat' as an argument
# to function parameter msg
print_argument('cat in the hat')
```

The following example has two parameters. Notice they are comma-separated. When calling the function, the program must pass two arguments. There is a one-to-one correspondence in the order of assigments. The first argument in the function call is assigned to the first parameter. The second argument in the function call is assigned to the second parameter. 

```
def print_names(fname, lname):
   print(fname + " " + lname)

# call function print_names. Pass two arguments to its parameters
# When calling the function, it expects two arguments because the 
# function was defined with two parameters. Try calling it with one or
# three parameters. You will get an error message.
print_names('Bart', 'Simpson')
```
<br><br>

### Arbitrary Arguments, *args

If you do not know how many arguments will be passed into a function, add a * before the parameter name in the function definition. The function will receive a *tuple* of arguments and can access the items accordingly.

Note this is an advanced concept which you will likely not code in the class, but will see *args in python library function definitions. The main takeaway is to understand that *args means you can pass an arbitrary number of arguments to the function.

```
# arbitrary arguments, *args
def some_function(*names):
   print('there are ' + str(len(names)) + ' names')
   for n in names:
      print(n)

# call some_function.
some_function('Bart', 'Homer', 'Marge', 'Lisa')
```
<br><br>

### Keyword Arguments, key=value

Arguments do not have to be in the same order as the parameters when using key = value syntax. Below, the value passed to c is defined in the function call as c = 9. Similarly, the values for b and a are defined with key=value syntax.

```
# keyword arguments
def average_of_three(a, b, c):
   avg = (a+b+c)/3
   print('average of a: {}, b: {}, c: {} is {}'.format(a,b,c,avg))

# call average_of_three, use key=value syntax
# does not require that we pass arguments in same order as parameters
average_of_three(c = 9, a = 7, b = 4)
```
<br><br>

### Arbitrary Keyword Arguments, **kwargs

If you do not know how many keyword arguments will be passed into your function, add two asterisk: ** before the parameter name in the function definition.

The function will receive a dictionary of arguments, and can access the items accordingly.

```
# aribtrary keyword arguments, **kwargs
def print_knames(**person):
   print("Last name is " + person["lname"])

# call function, specify keyword arugments
print_knames(fname='Charlie', lname = 'Brown')
```
<br><br>

#### Default Parameter Value

Default values may be assigned to a parameter, when the function is called without an argument.

```
# default parameter value
def my_university(school = "CU Denver"):
   print("My university is " + school)

# call function, use default value
my_university()

# call function, pass value to parameter
my_university("Montana State")

```
<br><br>

### Return Values

Use the return statement to return data. python allows returning multiple objects from a function. When returning more than one object, comma separate the list. See example below.

```
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
```
<br><br>

### pass statement

Programmers sometimes define a function before writing the function code. Writing a function with no statements in the body causes an error. The pass statement is used to avoid an error. It allows the function to be called.

```
