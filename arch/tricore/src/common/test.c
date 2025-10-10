char b;
char a = 5;

int tricore_live(char in)
{

	if (in > 0)
		return a;
	else
		tricore_live(1);

	return b;
}
