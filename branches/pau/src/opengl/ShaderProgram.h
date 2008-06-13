#ifndef __ShaderProgram_h__
#define __ShaderProgram_h__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <map>
#include "opengl.h"


class ShaderProgram
{
	GLuint vertexShaderID, fragmentShaderID, programID, oldProgramID;

public:

	bool init(const char *vertexCode, const char *fragCode)
	{
		// check that OpenGL 2.0 is available
		if(!opengl_init())
			return false;

		// allocate shaders
		vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
		fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

		// upload shaders' code
		glShaderSource(vertexShaderID, 1, &vertexCode,NULL);
		glShaderSource(fragmentShaderID, 1, &fragCode,NULL);

		// compile vertex shader
		glCompileShader(vertexShaderID);
		//if(glGetShader(vertexShaderID, GL_COMPILE_STATUS)==GL_FALSE))
		//	printf("Error compiling vertex shader\n");
		print_shader_info_log(vertexShaderID);

		// compile fragment shader
		glCompileShader(fragmentShaderID);
		print_shader_info_log(fragmentShaderID);

		// allocate the program
		programID = glCreateProgram();
		// attach the shaders
		glAttachShader(programID,vertexShaderID);
		glAttachShader(programID,fragmentShaderID);

		// link
		glLinkProgram(programID);

		print_program_info_log(programID);

		return true;
	}

	void begin() {
		GLint oldp;
		// record old program
		glGetIntegerv(GL_CURRENT_PROGRAM, &oldp);
		oldProgramID = (GLuint)oldp;

		// use this program
		glUseProgram(programID);

		// assert that the program is in use
		GLint newp;
		glGetIntegerv(GL_CURRENT_PROGRAM, &newp);
		if((int)programID!=newp)
			printf("ERROR USING PROGRAM %d.  Program %d will be used.\n",int(programID),int(newp));
	}
	void end() {
		glUseProgram(oldProgramID);
	}

	void release() {
		// clean up things
		glDeleteProgram(programID);
		glDeleteShader(fragmentShaderID);
		glDeleteShader(vertexShaderID);
	}

	void init_from_files( const char *vertexFile, const char *fragFile )
	{
		char *vs = text_file_read(vertexFile);
		char *fs = text_file_read(fragFile);

		init(vs,fs);

		free(vs);free(fs);
	}

	/// Read a text file into a newly allocated char*
	char *text_file_read(const char *fileName) {
		// open the file
		FILE *fp = fopen(fileName,"rt");
		if(fp == NULL) {
			printf("text file \"%s\" not found\n",fileName);
			return NULL;
		}

		// compute it's size
		fseek(fp, 0, SEEK_END);
		int count = ftell(fp);
		rewind(fp);

		// read the contents
		char *content = (char *)malloc(sizeof(char) * (count+1));
		count = fread(content,sizeof(char),count,fp);
		content[count] = '\0';

		fclose(fp);

		return content;
	}

	void print_shader_info_log(GLuint obj)
	{
		GLint infologLength = 0;
		GLint charsWritten  = 0;
		char *infoLog;

		glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

		if (infologLength > 0)
		{
			infoLog = (char *)malloc(infologLength);
			glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
			printf("%s",infoLog);
			free(infoLog);
		}
	}

	void print_program_info_log(GLuint obj)
	{
		GLint infologLength = 0;
		GLint charsWritten  = 0;
		char *infoLog;

		glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

		if(infologLength > 0)
		{
			infoLog = (char *)malloc(infologLength);
			glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
			printf("%s",infoLog);
			free(infoLog);
		}
	}

	GLint get_uniform(const char *name)
	{
		return glGetUniformLocation(programID,name);
	}
};

#endif //__ShaderProgram_h__

