from typing import AnyStr, NoReturn

# TODO: Raise Trace time exception
# Some assertions defines the same trace time if they have.
class TraceTimeError(Exception):
	pass
class IllegalTypeException(Exception):
	def __init__(self,var_name:AnyStr,line:int,col:int,origin_type:AnyStr,*expected_type:AnyStr) -> NoReturn:
		size=len(expected_type)
		expect=''
		for v in range(0,size-1):
			expect+=f'<class {expected_type[v]}> '
		if size==1:
			expect+=f'<class {expected_type[size-1]}>'
		else:
			expect+=f'or <class {expected_type[size-1]}>'
		msg=f'line:{line}:{col} \'{var_name}\' Illegal type ' \
			f':<class {origin_type}> Expect type:{expect}'
		super().__init__(msg)
