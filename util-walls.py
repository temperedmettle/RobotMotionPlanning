
import sys
def cell_number(walls):
	for wall in walls:
		print wall

if __name__ == '__main__':
	cell = 15
	for i in sys.argv:
		if i == 'l':
			cell = cell - 8
		if i == 'd':	
			cell = cell - 4
		if i == 'r':	
			cell = cell - 2
		if i == 'u':	
			cell = cell - 1
	print cell		
