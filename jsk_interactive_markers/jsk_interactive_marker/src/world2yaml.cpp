#include <tinyxml.h>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <sstream>

// tutorial demo program
//#include <stdafx.h>


// ----------------------------------------------------------------------
// STDOUT dump and indenting utility functions
// ----------------------------------------------------------------------
const unsigned int NUM_INDENTS_PER_SPACE=2;
std::ofstream ofs("test.txt");

const char * getIndent( unsigned int numIndents )
{
  static const char * pINDENT="                                      + ";
  static const unsigned int LENGTH=strlen( pINDENT );
  unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
  if ( n > LENGTH ) n = LENGTH;

  return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
  static const char * pINDENT="                                        ";
  static const unsigned int LENGTH=strlen( pINDENT );
  unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
  if ( n > LENGTH ) n = LENGTH;

  return &pINDENT[ LENGTH-n ];
}

int dump_attribs_to_stdout(TiXmlElement* pElement, unsigned int indent)
{
  if ( !pElement ) return 0;

  TiXmlAttribute* pAttrib=pElement->FirstAttribute();
  int i=0;
  int ival;
  double dval;
  const char* pIndent=getIndent(indent);
  printf("\n");
  while (pAttrib)
    {
      printf( "%s%s: value=[%s]", pIndent, pAttrib->Name(), pAttrib->Value());

      if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
      if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
      printf( "\n" );
      i++;
      pAttrib=pAttrib->Next();
    }
  return i;
}

void dump_include_model( TiXmlNode* pParent, unsigned int indent = 0 ){
  TiXmlNode* pChild;
  TiXmlText* pText;
  int t = pParent->Type();
  std::cout << "-" << std::endl;
  for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
    {
      TiXmlNode* pChild2;
      std::string type = pChild->Value();
      //std::cout << pChild->Value() << std::endl;

      if(type == "uri"){

	//std::cout << "uuuuuuuuuuurrrrrrrrrrrrrrrriiiiiiiiiiiiI" << std::endl;
	std::cout << getIndentAlt(indent) << "model: \"" << pChild->FirstChild()->Value() << "\"" << std::endl;
      }else if(type == "name"){
	//std::cout << "namerrrrrrrrrrrriiiiiiiiiiiiI" << std::endl;
	std::cout << getIndentAlt(indent) << "name: \"" << pChild->FirstChild()->Value() << "\"" << std::endl;
      }else if(type == "pose"){
	//	btQuaternion qua(
	std::string child_value = pChild->FirstChild()->Value();
	std::vector<std::string> v;
	boost::algorithm::split( v, child_value, boost::algorithm::is_space() );
	//std::cout << getIndentAlt(indent) << "pose: " << pChild->FirstChild()->Value() << std::endl;
	std::cout << getIndentAlt(indent) << "pose:" << std::endl;
	std::cout << getIndentAlt(indent + 1) << "position:" << std::endl;
	std::cout << getIndentAlt(indent + 2) << "x: " << v[0] << std::endl;
	std::cout << getIndentAlt(indent + 2) << "y: " << v[1] << std::endl;
	std::cout << getIndentAlt(indent + 2) << "z: " << v[2] << std::endl;
	
	std::string s = v[3];
	//std::istringstream r_str(s.c_str());
	std::istringstream r_str(v[3]);
	int r,p,y;
	r_str >> r;
	std::istringstream p_str(v[4]);
	p_str >> p;
	std::istringstream y_str(v[5]);
	y_str >> y;

	//	istr << v[4];
	//istr >> p;
	//istr << v[5];
	//istr >> y;

	//btQuaternion qua(y,p,r);
	//tf::Quaternion qua(y,p,r);
	tf::Matrix3x3 mat;
	mat.setRPY(r,p,y);
	tf::Quaternion q;
	mat.getRotation(q);
	std::cout << getIndentAlt(indent + 1) << "orientation:" << std::endl;
	//double x = q.x();
	std::cout << getIndentAlt(indent + 2) << "x: " << q.x() << std::endl;
	std::cout << getIndentAlt(indent + 2) << "y: " << q.y() << std::endl;
	std::cout << getIndentAlt(indent + 2) << "z: " << q.z() << std::endl;
	std::cout << getIndentAlt(indent + 2) << "w: " << q.w() << std::endl;
	//std::cout << getIndentAlt(indent + 2) << "y: " << q.y << std::endl;
	//std::cout << getIndentAlt(indent + 2) << "z: " << q.z << std::endl;

	
	/*
	for(int i=0; i< 3; i++){
	  std::cout << v[i] << "," << std::endl;
	  }*/
	//std::cout << "poserrrrrrrrrrrriiiiiiiiiiiiI" << std::endl;

      }
    }
  std::cout << getIndentAlt(indent) << "frmae-id: \"map\"" << std::endl;
  std::cout << getIndentAlt(indent) << "robot: false" << std::endl;
}


void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0 )
{
  if ( !pParent ) return;

  TiXmlNode* pChild;
  TiXmlText* pText;
  int t = pParent->Type();
  //printf( "%s", getIndent(indent));
  int num;
  //std::cout << "aaaaa"  << std::endl;
  //std::cout << pParent->Value() << std::endl;
  std::string value = pParent->Value();
  if ( value == "include"){
    //std::cout << "aaaaaaaaaaaaa" << std::endl;
    dump_include_model(pParent, 1);
  }

  /*
  switch ( t )
    {
    case TiXmlNode::TINYXML_DOCUMENT:
      printf( "Document" );
      break;

    case TiXmlNode::TINYXML_ELEMENT:
      printf( "Element [%s]", pParent->Value() );
      num=dump_attribs_to_stdout(pParent->ToElement(), indent+1);
      switch(num)
	{
	case 0:  printf( " (No attributes)"); break;
	case 1:  printf( "%s1 attribute", getIndentAlt(indent)); break;
	default: printf( "%s%d attributes", getIndentAlt(indent), num); break;
	}
      break;

    case TiXmlNode::TINYXML_COMMENT:
      printf( "Comment: [%s]", pParent->Value());
      break;

    case TiXmlNode::TINYXML_UNKNOWN:
      printf( "Unknown" );
      break;

    case TiXmlNode::TINYXML_TEXT:
      pText = pParent->ToText();
      printf( "Text: [%s]", pText->Value() );
      break;

    case TiXmlNode::TINYXML_DECLARATION:
      printf( "Declaration" );
      break;
    default:
      break;
    }
  printf( "\n" );
  */
  for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
    {
      dump_to_stdout( pChild, indent+1 );
    }
}

void dump_to_stdout(const char* pFilename)
{
  TiXmlDocument doc(pFilename);
  bool loadOkay = doc.LoadFile();
  if (loadOkay)
    {
      printf("\n%s:\n", pFilename);
      dump_to_stdout( &doc ); // defined later in the tutorial
    }
  else
    {
      printf("Failed to load file \"%s\"\n", pFilename);
    }
}



int main(int argc, char** argv){

  //ofs<<"aa"<<std::endl;
  dump_to_stdout("vrc_final_task1.world");

  return 0;


}

