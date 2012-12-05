/*==========================================================================*\
|                                                                            |
|                                                                            |
| PC-side Bootstrap Loader communication Application                         |
|                                                                            |
| See main.c for full version and legal information                          |
|                                                                            |
\*==========================================================================*/

#include "TextFileIO.h"

FILE* file;
unsigned int currentAddr = NO_DATA_READ;

/*******************************************************************************
*Function:    openTI_TextForRead
*Description: Opens a TXT file for reading
*Parameters: 
*             char *filename        The string containing the file name
*Returns:
*             SUCCESSFUL_OPERATION  The file is open for reading
*             ERROR_OPENING_FILE    Some error occured during file open
*******************************************************************************/
int openTI_TextForRead( char *filename )
{
   currentAddr = NO_DATA_READ;
   if( (file = fopen( filename, "rb" )) == 0 )
   {
      return ERROR_OPENING_FILE;
   }
   else
   {
      return OPERATION_SUCCESSFUL;
   }
}

/*******************************************************************************
*Function:    openTI_TextForWrite
*Description: Opens a TXT file for writing with append
*Parameters: 
*             char *filename        The string containing the file name
*Returns:
*             SUCCESSFUL_OPERATION  The file is open for reading
*             ERROR_OPENING_FILE    Some error occured during file open
*******************************************************************************/
int openTI_TextForWrite( char *filename )
{
   currentAddr = NO_DATA_READ;
   if( (file = fopen( filename, "a+" )) == 0 )
   {
      return ERROR_OPENING_FILE;
   }
   else
   {
      return OPERATION_SUCCESSFUL;
   }
}

/*******************************************************************************
*Function:    endTI_TextWrite
*Description: Writes the final 'q' to a TI TEXT file and closes it
*Parameters: 
*             none
*Returns:
*             none
*******************************************************************************/
void endTI_TextWrite()
{
  fprintf(file,"q\n");
  closeTI_Text();
}

/*******************************************************************************
*Function:    closeTI_Text
*Description: closes access to a TI TEXT file
*Parameters: 
*             none
*Returns:
*             none
*******************************************************************************/
void closeTI_Text()
{
   fclose( file );
}

/*******************************************************************************
*Function:    moreDataToRead
*Description: checks whether an end-of-file was hit during read
*Parameters: 
*             none
*Returns:
*             1                     if an EOF has not been hit
*             0                     if an EOF has been hit
*******************************************************************************/
int moreDataToRead()
{
  return !(currentAddr == TXT_EOF);
}

/*******************************************************************************
*Function:    writeTI_TextFile
*Description: writes a block of data in TI TEXT format to the current file
*Parameters: 
*             DataBlock data        The DataBlock structure to write
*Returns:
*             none
*******************************************************************************/
/*
void writeTI_TextFile( DataBlock data )
{

  unsigned int bytesWritten = 0;
  if( (currentAddr == NO_DATA_READ) || (currentAddr != data.startAddr) )
  {
    fprintf(file, "@%05X\n", data.startAddr);
	currentAddr = data.startAddr;
  }
  for( bytesWritten = 0; bytesWritten < data.numberOfBytes; bytesWritten++,currentAddr++ )
  {
    fprintf(file, "%02X", data.data[bytesWritten]);
	if( ((bytesWritten+1)%16 == 0) || (bytesWritten+1 == data.numberOfBytes) )
	{
      fprintf(file, "\n");
	} // if
	else
	{
      fprintf(file, " ");
	}
  }
}
*/
/*******************************************************************************
*Function:    writeTI_TextFile
*Description: writes a block of data in TI TEXT format to the current file
*Parameters: 
*             int length                 The address of bytes
*             unsigned char *data        The array to write
*             int length                 The amount of bytes
*Returns:
*             none
*******************************************************************************/
void writeTI_TextFile( int addr, unsigned char *data, int length )
{
  int i;
   fprintf(file, "@%05X", addr);
  for( i = 0; i < length; i++)
  {
    if( i%16 == 0 )
	{
      fprintf(file, "\n");
	}
	else
	{
      fprintf(file, " ");
	}

    fprintf(file, "%02X", data[i]);
	/*
	if( i == 0 )
	{
      fprintf(file, " ");
    }
	else if( (i%16 == 0) || (i == length-1) || (i == 15))
	{
      fprintf(file, "\n");
	}
	else
	{
      fprintf(file, " ");
	}
	*/
  }
  fprintf(file, "\n");

}
/*******************************************************************************
*Function:    v
*Description: reads a certain amount of bytes from a TI TEXT file
*Parameters: 
*             int bytesToRead       The maximum number of bytes to read from the file
*Returns:
*             A Datablock structure with the requested bytes
*******************************************************************************/
DataBlock readTI_TextFile(int bytesToRead)
{
  DataBlock returnBlock;
  int bytesRead = 0;
  char string[50];
  int status;
  if( currentAddr == NO_DATA_READ )
  {
    fgets( string, sizeof string, file );
    sscanf(&string[1], "%x\n", &currentAddr);
  }
  returnBlock.startAddr = currentAddr;
  do
  {
	int stringLength=0;
	int stringPosition=0;
    status = fgets( string, sizeof string, file );
	stringLength = strlen( string );
	if( status == 0 )
	{
      currentAddr = EOF;
	}
    else if( string[0] == '@' )
	{
      sscanf(&string[1], "%x\n", &currentAddr);
	  status = 0;
	}
	else if ( string[0] == 'q' || string[0] == 'Q' )
	{
      status = 0;
	  currentAddr = EOF;
	}
    else
	{
	  for( stringPosition = 0; stringPosition < (stringLength-3); stringPosition+=3 )
	  {
        sscanf( &string[stringPosition], "%2x", &returnBlock.data[bytesRead] );
	    bytesRead++;
		currentAddr++;
	  }
	}
  }
  while( (status != 0) && (bytesRead < bytesToRead) );
  returnBlock.numberOfBytes = bytesRead;
  return returnBlock;
}
