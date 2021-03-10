import sys

sys.path.append("/home/ronaldo/Desktop/projects/AutonomousDrivingTest")
from src.scenest_parser.ast import ASTDumper,Parse
import sys
if __name__ == "__main__":
	assert len(sys.argv)==2
	ast=Parse(sys.argv[1])
	dumper=ASTDumper(ast)
	dumper.dump()
	