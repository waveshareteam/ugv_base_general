// funcs for editing the files in flash.

bool flashStatus = false;

// initialize littleFS for flash file system ctrl.
void initFS() {
	if (!LittleFS.begin(true)){
		if (InfoPrint == 1) {Serial.println("LittleFS mount failed.");}
		flashStatus = false;
	}
	else {
		if (InfoPrint == 1) {Serial.println("LittleFS mount succeed.");}
		flashStatus = true;
	}
}


// get the free space of flash.
uint32_t freeFlashSpace(){
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  uint32_t freeSpace = total - used;
  if (InfoPrint == 1) {
  	Serial.print("totalBytes:\t");Serial.print(total);
  	Serial.println(" bytes");
  	Serial.print("free flash memory:\t");Serial.print(freeSpace);
		Serial.println(" bytes");
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["info"] = "free flash space";
  jsonInfoHttp["total"] = total;
  jsonInfoHttp["free"] = freeSpace;
  
  return freeSpace; 
}


// scan all the files saved in flash.
void scanFlashContents() {
		jsonInfoHttp.clear();
    
    File root = LittleFS.open("/");
    if (!root.isDirectory()) {
    	if (InfoPrint == 1) {
    		Serial.println("error: not a directory.");
    		jsonInfoHttp["info"] = "error: not a directory.";
    		return;
    	}
    }

    jsonInfoHttp["info"] = "reading files and the first line";
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
        	Serial.println(">>>---=== File Name and First line ===---<<<");
        	Serial.println("[file]: [" + String(file.name()) + "]");
        	Serial.println("[first line]:");
        	String line = file.readStringUntil('\n');

		    		if (line) {
		    			Serial.println(line);
		    			jsonInfoHttp[file.name()] = line;
		    		} else {
		    			Serial.println("no content.");
		    			jsonInfoHttp[file.name()] = "[null]";
		    		}
		        file.close();
        } else if (file.isDirectory()) {
        	if (file) {
        		Serial.println("Failed to open file: " + String(file.name()));
        		jsonInfoHttp[file.name()] = "[failed to open]";
        	}
        }
        file = root.openNextFile();
    }
}


// create a new file and input the content.
bool createFile(String fileName, String fileContent) {
	jsonInfoHttp.clear();
	if (!flashStatus) {
		if (InfoPrint == 1) {Serial.println("LittleFS mount failed.");}
  	jsonInfoHttp["info"] = "LittleFS mount failed.";
		return false;
	}

	if (LittleFS.exists("/"+fileName)) {
		if (InfoPrint == 1) {Serial.println("file already exists.");}
		jsonInfoHttp["info"] = "file already exists.";
		return false;
	}

	File file = LittleFS.open("/"+fileName, "w");
	if (file) {
		// file.println("{\"name\":\"" + fileName + "\",\"intro\":\"" + fileContent + "\"}");
		file.println(fileContent);
		file.close();
		if (InfoPrint == 1) {Serial.println("file created successfully.");}
		jsonInfoHttp["info"] = "file created successfully.";
		return true;
	} else {
		if (InfoPrint == 1) {Serial.println("file creation failed.");}
		jsonInfoHttp["info"] = "file creation failed.";
		return false;
	}
}


// read a file, this function return the lineNum.
int readFile(String fileName) {
  jsonInfoHttp.clear();
  jsonInfoHttp["info"] = "reading file";
	File file = LittleFS.open("/" + fileName, "r");
	if (!file) {
		Serial.println("file not found.");
		jsonInfoHttp["info"] = "file not found";
		return -1;
	}

	Serial.println("---=== File Content ===---");
	Serial.println("reading file: [" + fileName + "] starts:");
	
	jsonInfoHttp["name"] = fileName;

	int _LineNum = -1;
	while (file.available()) {
		_LineNum++;
		String line = file.readStringUntil('\n');
		Serial.print("[lineNum: ");Serial.print(_LineNum+1);Serial.print(" ] - ");
		Serial.println(line);

		jsonInfoHttp["lineNum_"+String(_LineNum+1)] = line;
	}

	Serial.println("^^^ ^^^ ^^^ reading file: " + fileName + " ends. ^^^ ^^^ ^^^");
	file.close();

	return _LineNum + 1;
}


// delete a file.
bool deleteFile(String inputName) {
	jsonInfoHttp.clear();
	if (!flashStatus) {
		if (InfoPrint == 1) {Serial.println("LittleFS mount failed.");}
		jsonInfoHttp["info"] = "LittleFS mount failed.";
		return false;
	}

	if (!LittleFS.exists("/" + inputName)) {
		if (InfoPrint == 1) {Serial.println("file already deleted.");}
		jsonInfoHttp["info"] = "file already deleted.";
		return false;
	}

	LittleFS.remove("/" + inputName);
	if (InfoPrint == 1) {Serial.println("file deleted successfully.");}
	jsonInfoHttp["info"] = "file deleted successfully.";
	return true;
}


// add content at the end of a file.
void appendLine(String fileName, String appendContent) {
	Serial.println("--- --- --- RAW FILE -- --- ---");
	if (readFile(fileName) == -1) {return;}

	File file = LittleFS.open("/" + fileName, "a");

	if(!file){
	Serial.println("Error opening file for appending.");
	return;
	}

	file.println(appendContent);
	file.close();

	Serial.println("--- --- --- NEW FILE -- --- ---");
	jsonInfoHttp.clear();
	readFile(fileName);
}


// insert a new line under the lineNum.
void insertLine(String filename, int lineNum, String newLineString) {
	Serial.println("--- --- --- RAW FILE -- --- ---");
	int _LineNum = readFile(filename);
	if (_LineNum == -1) {return;}

	String lines[_LineNum+1];

	File file = LittleFS.open("/" + filename, "r");
	if(!file){
		Serial.println("Error opening file for writing.");
		return;
	}

	int i = 0;
	while (file.available()) {
		if (i == lineNum - 1) {
			lines[i] = newLineString;
			i++;
		}
		lines[i] = file.readStringUntil('\n');
		i++;
		}
	file.close();

	file = LittleFS.open("/" + filename, "w");
	for(int j=0; j<i; j++){
		file.println(lines[j]);
	}
	file.close();

	Serial.println("--- --- --- NEW FILE -- --- ---");
	jsonInfoHttp.clear();
	readFile(filename);
}


// change a single line in the file.
void replaceLine(String filename, int lineNum, String newLineString) {
	Serial.println("--- --- --- RAW FILE -- --- ---");
	int _LineNum = readFile(filename);
	if (_LineNum == -1) {return;}

	String lines[_LineNum]; 

	File file = LittleFS.open("/" + filename, "r");
	if(!file){
		Serial.println("Error opening file.");
		return;
	}  

	int i = 0;
	while (file.available()) {
		lines[i] = file.readStringUntil('\n');
		i++;
	}
	file.close();

	lines[lineNum-1] = newLineString;

	file = LittleFS.open("/" + filename, "w");
	for(int j=0; j<i; j++){
		file.println(lines[j]);
	}
	file.close();

	Serial.println("--- --- --- NEW FILE -- --- ---");
	jsonInfoHttp.clear();
	readFile(filename);
}


// read a single line from file.
String readSingleLine(String filename, int lineNum) {
	File file = LittleFS.open("/" + filename, "r");
	if(!file){
		Serial.println("Error opening file.");
		return "";
	}  
	jsonInfoHttp.clear();
	String line;
	int i = 0;
	while (file.available()) {
		line = file.readStringUntil('\n');
		if (i == lineNum-1) {
			file.close();
			if (InfoPrint == 1) {
				Serial.println(line);
				jsonInfoHttp["filename"] = filename;
				jsonInfoHttp["lineNum"]  = lineNum;
			}
			return line;
		}
		i++;
	}
	file.close();
	if (InfoPrint == 1) {
		Serial.println("[line not found]");
	}
	return "";
}


void deleteSingleLine(String fileName, int lineNum){
  Serial.println("--- --- --- RAW FILE -- --- ---");
  File file = LittleFS.open("/" + fileName, "r+");
  readFile(fileName);
  if(!file){
    Serial.println("Error opening file for reading");
    return;
  }

  String contents = "";
  int i = 1;

  while(file.available()){
    String line = file.readStringUntil('\n');
    if(i != lineNum){
      contents += line + "\n";  
    }
    i++;
  }

  file.close();

  file = LittleFS.open("/" + fileName, "w");
  file.print(contents);
  file.close();

  Serial.println("--- --- --- NEW FILE -- --- ---");
  jsonInfoHttp.clear();
  readFile(fileName);
}