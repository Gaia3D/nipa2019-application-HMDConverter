#pragma once

#ifdef AVEVAFORMAT

#include "aReader.h"

class AvevaRevReader : public aReader
{
public:
	AvevaRevReader();
	virtual ~AvevaRevReader();

public:
	virtual bool readRawDataFile(std::string& filePath);

	virtual void clear();
};

#endif

