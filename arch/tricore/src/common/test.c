char a;

int tricore_live(char in)
{

	if (in > 0)
		return a;
	else
		tricore_live(1);

	return a;
}
