# Lesson 7 - python lists

A list is a data structure in Python that is a mutable, or changeable, ordered sequence of elements. Each element or value that is inside of a list is called an item. Just as strings are defined as characters between quotes, lists are defined by having values between square brackets [ ].

A list is not the same as an array, as arrays contain objects of the same data type. Python does not have built-in support for arrays. In a later lesson, we will learn how to use the numpy library for arrays.

https://docs.python.org/3/library/stdtypes.html#typesseq-list 


### lesson7a.py - Getting started with lists

- Create a list that contains string data types. 
- Print the object's data type. 
- Print the list contents

Lists may be constructed in several ways. The example below creates a list using square brackets, separating items with commas: [a, b, c]

```
# create a list that contains string data types
school_supplies = ['pencil', 'paper', 'glue', 'calculator', 'notebook']

# what is the school_supplies data type?
print("type(school_supplies) ", type(school_supplies))

# print all items in the list
print(school_supplies)
```
<br>

**output**

```
type(school_supplies)  <class 'list'>
['pencil', 'paper', 'glue', 'calculator', 'notebook']
```
<br>

Each item in a list corresponds to an index number, which is an integer value, starting with the index number 0.

For the list school_supplies, the index breakdown looks like this:
| 'pencil' | 'paper' | 'glue' | 'calculator' | 'notebook' |
| --- | --- | --- | --- | --- |
| 0 | 1 | 2 | 3 | 4 |


The first item, the string 'pencil' starts at index 0, and the list ends at index 4 with the item 'notebook'.

Because each item in a Python list has a corresponding index number, we’re able to access and manipulate lists in the same ways we can with other sequential data types.

Now we can access a discrete item of the list by referring to its index number:

```
# access list items by index number
print("\naccess list item at index 2")
print(school_supplies[2])
school_supplies[2] = 'paste'
print(school_supplies[2])
```

**output**
```
access list item at index 2
glue
paste

```

A for loop can be used to iterate through the list sequence.

```
for iterating_var in sequence:
   statements(s)
```

s is the iterator variable that provides access to the list item.

```
# iterate through the list, access each object individually
print("\niterating through list with for loop")
for s in school_supplies:
   print(s)
```

**output**
```
iterating through list with for loop
pencil
paper
paste
calculator
notebook

```

### lesson7b.py - list may contain different data types


### lesson7c.py - list methods

The list data type has several methods. See https://docs.python.org/3/tutorial/datastructures.html for the available methods of list objects. 

The example below demonstrates the usage of the append, insert, and remove methods.




