#include"Compressor.h"
#include"Solenoid.h"


class Piston 
{
public:
		Piston();
		~Piston();
		
		void Open();
		void Close();

private:
		Compressor*pCompressor;
		Solenoid*pSolinoid;
};
