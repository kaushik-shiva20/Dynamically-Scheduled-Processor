#include "sim_ooo.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <iomanip>
#include <map>


using namespace std;

//used for debugging purposes
static const char *stage_names[NUM_STAGES] = {"ISSUE", "EXE", "WR", "COMMIT"};
static const char *instr_names[NUM_OPCODES] = {"LW", "SW", "ADD", "ADDI", "SUB", "SUBI", "XOR", "AND", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};
static const char *res_station_names[5]={"Int", "Add", "Mult", "Load"};

unsigned mBaseAddr;
unsigned mDataMemSize;
unsigned mROBTotalEntries;
bool isBranchMispredicted = false;
unsigned currClkCycle;
instruction_t *mInstrMemPtr;
sim_ooo * currSim;
unit_t mDummyExeUnit[MAX_UNITS];
unsigned mNumDummyUnits;
unsigned mCurrDummyUnitIndex = 0;
/* =============================================================

   HELPER FUNCTIONS (misc)

   ============================================================= */
inline bool isValidPC(unsigned mPC);
inline bool isLoadInstr(opcode_t mOpCode);
inline bool isStoreInstr(opcode_t mOpCode);
bool isOpcodeFpType(opcode_t mOpCode);
void sim_Commit_Handler(sim_ooo * mSim);
void sim_WB_Handler(sim_ooo * mSim);
void sim_Exe_Handler(sim_ooo * mSim);
void sim_Issue_Handler(sim_ooo * mSim);
unsigned search_exe_unit(unsigned mPC);
void update_instr_window(unsigned mPC, stage_t mStage);
void store_bypassing_wb_handler(unsigned mPC);
unsigned search_prev_load_store(res_station_entry_t * mStation);

/* convert a float into an unsigned */
inline unsigned float2unsigned(float value){
	unsigned result;
	memcpy(&result, &value, sizeof value);
	return result;
}

/* convert an unsigned into a float */
inline float unsigned2float(unsigned value){
	float result;
	memcpy(&result, &value, sizeof value);
	return result;
}

/* convert integer into array of unsigned char - little indian */
inline void unsigned2char(unsigned value, unsigned char *buffer){
        buffer[0] = value & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
}

/* convert array of char into integer - little indian */
inline unsigned char2unsigned(unsigned char *buffer){
       return buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
}

/* the following six functions return the kind of the considered opcdoe */

bool is_branch(opcode_t opcode){
        return (opcode == BEQZ || opcode == BNEZ || opcode == BLTZ || opcode == BLEZ || opcode == BGTZ || opcode == BGEZ || opcode == JUMP);
}

bool is_memory(opcode_t opcode){
        return (opcode == LW || opcode == SW || opcode == LWS || opcode == SWS);
}

bool is_int_r(opcode_t opcode){
        return (opcode == ADD || opcode == SUB || opcode == MULT || opcode == DIV || opcode == XOR || opcode == AND);
}

bool is_int_imm(opcode_t opcode){
        return (opcode == ADDI || opcode == SUBI );
}

bool is_int(opcode_t opcode){
        return (is_int_r(opcode) || is_int_imm(opcode));
}

bool is_fp_alu(opcode_t opcode){
        return (opcode == ADDS || opcode == SUBS || opcode == MULTS || opcode == DIVS);
}

/* clears a ROB entry */
void clean_rob(rob_entry_t *entry){
        entry->ready=false;
        entry->pc=UNDEFINED;
        entry->state=ISSUE;
        entry->destination=UNDEFINED;
        entry->value=UNDEFINED;
}

/* clears a reservation station */
void clean_res_station(res_station_entry_t *entry){
        entry->pc=UNDEFINED;
        entry->value1=UNDEFINED;
        entry->value2=UNDEFINED;
        entry->tag1=UNDEFINED;
        entry->tag2=UNDEFINED;
        entry->destination=UNDEFINED;
        entry->address=UNDEFINED;
        entry->CDBWriteDataAvailClkCycle= -1;
        entry->CDBWriteDataAvailClkCyclevalue2 = -1;
}

/* clears an entry if the instruction window */
void clean_instr_window(instr_window_entry_t *entry){
        entry->pc=UNDEFINED;
        entry->issue=UNDEFINED;
        entry->exe=UNDEFINED;
        entry->wr=UNDEFINED;
        entry->commit=UNDEFINED;
}

/* implements the ALU operation 
   NOTE: this function does not cover LOADS and STORES!
*/
unsigned alu(opcode_t opcode, unsigned value1, unsigned value2, unsigned immediate, unsigned pc){
	unsigned result;
	switch(opcode){
			case ADD:
			case ADDI:
				result = value1+value2;
				break;
			case SUB:
			case SUBI:
				result = value1-value2;
				break;
			case XOR:
				result = value1 ^ value2;
				break;
			case AND:
				result = value1 & value2;
				break;
			case MULT:
				result = value1 * value2;
				break;
			case DIV:
				result = value1 / value2;
				break;
			case ADDS:
				result = float2unsigned(unsigned2float(value1) + unsigned2float(value2));
				break;
			case SUBS:
				result = float2unsigned(unsigned2float(value1) - unsigned2float(value2));
				break;
			case MULTS:
				result = float2unsigned(unsigned2float(value1) * unsigned2float(value2));
				break;
			case DIVS:
				result = float2unsigned(unsigned2float(value1) / unsigned2float(value2));
				break;
			case JUMP:
				result = pc + 4 + immediate;
				break;
			default: //branches
				int reg = (int) value1;
				bool condition = ((opcode == BEQZ && reg==0) ||
				(opcode == BNEZ && reg!=0) ||
  				(opcode == BGEZ && reg>=0) ||
  				(opcode == BLEZ && reg<=0) ||      
  				(opcode == BGTZ && reg>0) ||       
  				(opcode == BLTZ && reg<0));
				if (condition)
	 				result = pc+4+immediate;
				else 
					result = pc+4;
				break;
	}
	return 	result;
}

/* writes the data memory at the specified address */
void sim_ooo::write_memory(unsigned address, unsigned value){
	unsigned2char(value,data_memory+address);
}

/* =============================================================

   Handling of FUNCTIONAL UNITS

   ============================================================= */

/* initializes an execution unit */
void sim_ooo::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
        for (unsigned i=0; i<instances; i++){
                exec_units[num_units].type = exec_unit;
                exec_units[num_units].latency = latency;
                if(exec_units[num_units].type == MEMORY)
                {
                    //exec_units[num_units].latency--;
                }
                exec_units[num_units].busy = 0;
                exec_units[num_units].pc = UNDEFINED;
                exec_units[num_units].isAvailable = true;

                mDummyExeUnit[num_units].pc = UNDEFINED;
                mDummyExeUnit[num_units].isAvailable = true;
                mDummyExeUnit[num_units].latency = 1;
                mDummyExeUnit[num_units].busy = 0;
                mDummyExeUnit[num_units].reservationStationIndex = UNDEFINED;
                num_units++;
                mNumDummyUnits++;
        }

}

/* returns a free unit for that particular operation or UNDEFINED if no unit is currently available */
unsigned sim_ooo::get_free_unit(opcode_t opcode){
	if (num_units == 0){
		cout << "ERROR:: simulator does not have any execution units!\n";
		exit(-1);
	}
	for (unsigned u=0; u<num_units; u++){
		switch(opcode){
			//Integer unit
			case ADD:
			case ADDI:
			case SUB:
			case SUBI:
			case XOR:
			case AND:
			case BEQZ:
			case BNEZ:
			case BLTZ:
			case BGTZ:
			case BLEZ:
			case BGEZ:
			case JUMP:
				if (exec_units[u].isAvailable==true && exec_units[u].type==INTEGER && exec_units[u].busy==0 && exec_units[u].pc==UNDEFINED) return u;
				break;
			//memory unit
			case LW:
			case SW:
			case LWS: 
			case SWS:
				if (exec_units[u].isAvailable==true && exec_units[u].type==MEMORY && exec_units[u].busy==0 && exec_units[u].pc==UNDEFINED) return u;
				break;
			// FP adder
			case ADDS:
			case SUBS:
				if (exec_units[u].isAvailable==true && exec_units[u].type==ADDER && exec_units[u].busy==0 && exec_units[u].pc==UNDEFINED) return u;
				break;
			// Multiplier
			case MULT:
			case MULTS:
				if (exec_units[u].isAvailable==true && exec_units[u].type==MULTIPLIER && exec_units[u].busy==0 && exec_units[u].pc==UNDEFINED) return u;
				break;
			// Divider
			case DIV:
			case DIVS:
				if (exec_units[u].isAvailable==true && exec_units[u].type==DIVIDER && exec_units[u].busy==0 && exec_units[u].pc==UNDEFINED) return u;
				break;
			default:
				cout << "ERROR:: operations not requiring exec unit!\n";
				exit(-1);
		}
	}
	return UNDEFINED;
}



/* ============================================================================

   Primitives used to print out the state of each component of the processor:
   	- registers
	- data memory
	- instruction window
        - reservation stations and load buffers
        - (cycle-by-cycle) execution log
	- execution statistics (CPI, # instructions executed, # clock cycles) 

   =========================================================================== */
 

/* prints the content of the data memory */
void sim_ooo::print_memory(unsigned start_address, unsigned end_address){
	cout << "DATA MEMORY[0x" << hex << setw(8) << setfill('0') << start_address << ":0x" << hex << setw(8) << setfill('0') <<  end_address << "]" << endl;
	for (unsigned i=start_address; i<end_address; i++){
		if (i%4 == 0) cout << "0x" << hex << setw(8) << setfill('0') << i << ": "; 
		cout << hex << setw(2) << setfill('0') << int(data_memory[i]) << " ";
		if (i%4 == 3){
			cout << endl;
		}
	} 
}

/* prints the value of the registers */
void sim_ooo::print_registers(){
        unsigned i;
	cout << "GENERAL PURPOSE REGISTERS" << endl;
	cout << setfill(' ') << setw(8) << "Register" << setw(22) << "Value" << setw(5) << "ROB" << endl;
        for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_int_register_tag(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(22) << "-" << setw(5) << get_int_register_tag(i) << endl;
                else if (get_int_register(i)!=(int)UNDEFINED) 
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(11) << get_int_register(i) << hex << "/0x" << setw(8) << setfill('0') << get_int_register(i) << setfill(' ') << setw(5) << "-" << endl;
        }
	for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_fp_register_tag(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(22) << "-" << setw(5) << get_fp_register_tag(i) << endl;
                else if (float2unsigned(get_fp_register(i)) != UNDEFINED)
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(11) << get_fp_register(i) << hex << "/0x" << setw(8) << setfill('0') << float2unsigned(get_fp_register(i)) << setfill(' ') << setw(5) << "-" << endl;
	}
	cout << endl;
}

/* prints the content of the ROB */
void sim_ooo::print_rob(){
	cout << "REORDER BUFFER" << endl;
	cout << setfill(' ') << setw(5) << "Entry" << setw(6) << "Busy" << setw(7) << "Ready" << setw(12) << "PC" << setw(10) << "State" << setw(6) << "Dest" << setw(12) << "Value" << endl;
	for(unsigned i=0; i< rob->num_entries;i++){
		rob_entry_t entry = rob->entries[i];
		instruction_t instruction;
		if (entry.pc != UNDEFINED) instruction = instr_memory[(entry.pc-instr_base_address)>>2]; 
		cout << setfill(' ');
		cout << setw(5) << i;
		cout << setw(6);
		if (entry.pc==UNDEFINED) cout << "no"; else cout << "yes";
		cout << setw(7);
		if (entry.ready) cout << "yes"; else cout << "no";	
		if (entry.pc!= UNDEFINED ) cout << "  0x" << hex << setfill('0') << setw(8) << entry.pc;
		else	cout << setw(12) << "-";
		cout << setfill(' ') << setw(10);
		if (entry.pc==UNDEFINED) cout << "-";		
		else cout << stage_names[entry.state];
		if (entry.destination==UNDEFINED) cout << setw(6) << "-";
		else{
			if (instruction.opcode == SW || instruction.opcode == SWS)
				cout << setw(6) << dec << entry.destination; 
			else if (entry.destination < NUM_GP_REGISTERS && (!isOpcodeFpType(instruction.opcode)))
				cout << setw(5) << "R" << dec << entry.destination;
			else
				cout << setw(5) << "F" << dec << entry.destination;
		}
		if (entry.value!=UNDEFINED) cout << "  0x" << hex << setw(8) << setfill('0') << entry.value << endl;	
		else cout << setw(12) << setfill(' ') << "-" << endl;
	}
	cout << endl;
}

/* prints the content of the reservation stations */
void sim_ooo::print_reservation_stations(){
	cout << "RESERVATION STATIONS" << endl;
	cout  << setfill(' ');
	cout << setw(7) << "Name" << setw(6) << "Busy" << setw(12) << "PC" << setw(12) << "Vj" << setw(12) << "Vk" << setw(6) << "Qj" << setw(6) << "Qk" << setw(6) << "Dest" << setw(12) << "Address" << endl; 
	for(unsigned i=0; i< reservation_stations->num_entries;i++){
		res_station_entry_t entry = reservation_stations->entries[i];
	 	cout  << setfill(' ');
		cout << setw(6); 
		cout << res_station_names[entry.type];
		cout << entry.name + 1;
		cout << setw(6);
		if (entry.pc==UNDEFINED) cout << "no"; else cout << "yes";
		if (entry.pc!= UNDEFINED ) cout << setw(4) << "  0x" << hex << setfill('0') << setw(8) << entry.pc;
		else	cout << setfill(' ') << setw(12) <<  "-";			
		if (entry.value1!= UNDEFINED ) cout << "  0x" << setfill('0') << setw(8) << hex << entry.value1;
		else	cout << setfill(' ') << setw(12) << "-";			
		if (entry.value2!= UNDEFINED ) {
            if(isLoadInstr(entry.entry_instr.opcode))
            {
                if((entry.CDBWriteDataAvailClkCycle+1) < (int)(currClkCycle))
                {
                    cout << "  0x" << setfill('0') << setw(8) << hex << entry.value2;
                }else
                {
                    cout << setfill(' ') << setw(12) << "-";
                }
            }else {
                cout << "  0x" << setfill('0') << setw(8) << hex << entry.value2;
            }
        }
		else	cout << setfill(' ') << setw(12) << "-";			
		cout << setfill(' ');
		cout <<setw(6);
		if (entry.tag1!= UNDEFINED ) cout << dec << entry.tag1;
		else	cout << "-";			
		cout <<setw(6);
		if (entry.tag2!= UNDEFINED ) cout << dec << entry.tag2;
		else	cout << "-";			
		cout <<setw(6);
		if (entry.destination!= UNDEFINED ) cout << dec << entry.destination;
		else	cout << "-";			
		if (entry.address != UNDEFINED ) cout <<setw(4) << "  0x" << setfill('0') << setw(8) << hex << entry.address;
		else	cout << setfill(' ') << setw(12) <<  "-";			
		cout << endl;	
	}
	cout << endl;
}

/* prints the state of the pending instructions */
void sim_ooo::print_pending_instructions(){
	cout << "PENDING INSTRUCTIONS STATUS" << endl;
	cout << setfill(' ');
	cout << setw(10) << "PC" << setw(7) << "Issue" << setw(7) << "Exe" << setw(7) << "WR" << setw(7) << "Commit";
	cout << endl;
	for(unsigned i=0; i< pending_instructions.num_entries;i++){
		instr_window_entry_t entry = pending_instructions.entries[i];
		if (entry.pc!= UNDEFINED ) cout << "0x" << setfill('0') << setw(8) << hex << entry.pc;
		else	cout << setfill(' ') << setw(10)  << "-";
		cout << setfill(' ');
		cout << setw(7);			
		if (entry.issue!= UNDEFINED ) cout << dec << entry.issue;
		else	cout << "-";			
		cout << setw(7);			
		if (entry.exe!= UNDEFINED ) cout << dec << entry.exe;
		else	cout << "-";			
		cout << setw(7);			
		if (entry.wr!= UNDEFINED ) cout << dec << entry.wr;
		else	cout << "-";			
		cout << setw(7);			
		if (entry.commit!= UNDEFINED ) cout << dec << entry.commit;
		else	cout << "-";
		cout << endl;			
	}
	cout << endl;
}


/* initializes the execution log */
void sim_ooo::init_log(){
	log << "EXECUTION LOG" << endl;
	log << setfill(' ');
	log << setw(10) << "PC" << setw(7) << "Issue" << setw(7) << "Exe" << setw(7) << "WR" << setw(7) << "Commit";
	log << endl;
}

/* adds an instruction to the log */
void sim_ooo::commit_to_log(instr_window_entry_t entry){
                if (entry.pc!= UNDEFINED ) log << "0x" << setfill('0') << setw(8) << hex << entry.pc;
                else    log << setfill(' ') << setw(10)  << "-";
                log << setfill(' ');
                log << setw(7);
                if (entry.issue!= UNDEFINED ) log << dec << entry.issue;
                else    log << "-";
                log << setw(7);
                if (entry.exe!= UNDEFINED ) log << dec << entry.exe;
                else    log << "-";
                log << setw(7);
                if (entry.wr!= UNDEFINED ) log << dec << entry.wr;
                else    log << "-";
                log << setw(7);
                if (entry.commit!= UNDEFINED ) log << dec << entry.commit;
                else    log << "-";
                log << endl;
}

/* prints the content of the log */
void sim_ooo::print_log(){
	cout << log.str();
}

/* prints the state of the pending instruction, the content of the ROB, the content of the reservation stations and of the registers */
void sim_ooo::print_status(){
	print_pending_instructions();
	print_rob();
	print_reservation_stations();
	print_registers();
}

/* execution statistics */

float sim_ooo::get_IPC(){return (float)instructions_executed/clock_cycles;}

unsigned sim_ooo::get_instructions_executed(){return instructions_executed;}

unsigned sim_ooo::get_clock_cycles(){return clock_cycles;}



/* ============================================================================

   PARSER

   =========================================================================== */


void sim_ooo::load_program(const char *filename, unsigned base_address){

   /* initializing the base instruction address */
   instr_base_address = base_address;
    mBaseAddr = instr_base_address;
    PC = instr_base_address;
    mInstrMemPtr = instr_memory;
   /* creating a map with the valid opcodes and with the valid labels */
   map<string, opcode_t> opcodes; //for opcodes
   map<string, unsigned> labels;  //for branches
   for (int i=0; i<NUM_OPCODES; i++)
	 opcodes[string(instr_names[i])]=(opcode_t)i;

   /* opening the assembly file */
   ifstream fin(filename, ios::in | ios::binary);
   if (!fin.is_open()) {
      cerr << "error: open file " << filename << " failed!" << endl;
      exit(-1);
   }

   /* parsing the assembly file line by line */
   string line;
   unsigned instruction_nr = 0;
   while (getline(fin,line)){
	
	// set the instruction field
	char *str = const_cast<char*>(line.c_str());

  	// tokenize the instruction
	char *token = strtok (str," \t");
	map<string, opcode_t>::iterator search = opcodes.find(token);
        if (search == opcodes.end()){
		// this is a label for a branch - extract it and save it in the labels map
		string label = string(token).substr(0, string(token).length() - 1);
		labels[label]=instruction_nr;
		// move to next token, which must be the instruction opcode
		token = strtok (NULL, " \t");
		search = opcodes.find(token);
		if (search == opcodes.end()) cout << "ERROR: invalid opcode: " << token << " !" << endl;
	}

	instr_memory[instruction_nr].opcode = search->second;

	//reading remaining parameters
	char *par1;
	char *par2;
	char *par3;
	switch(instr_memory[instruction_nr].opcode){
		case ADD:
		case SUB:
		case XOR:
		case AND:
		case MULT:
		case DIV:
		case ADDS:
		case SUBS:
		case MULTS:
		case DIVS:
			par1 = strtok (NULL, " \t");
			par2 = strtok (NULL, " \t");
			par3 = strtok (NULL, " \t");
			instr_memory[instruction_nr].dest = atoi(strtok(par1, "RF"));
			instr_memory[instruction_nr].src1 = atoi(strtok(par2, "RF"));
			instr_memory[instruction_nr].src2 = atoi(strtok(par3, "RF"));
			break;
		case ADDI:
		case SUBI:
			par1 = strtok (NULL, " \t");
			par2 = strtok (NULL, " \t");
			par3 = strtok (NULL, " \t");
			instr_memory[instruction_nr].dest = atoi(strtok(par1, "R"));
			instr_memory[instruction_nr].src1 = atoi(strtok(par2, "R"));
			instr_memory[instruction_nr].immediate = strtoul (par3, NULL, 0); 
			break;
		case LW:
		case LWS:
			par1 = strtok (NULL, " \t");
			par2 = strtok (NULL, " \t");
			instr_memory[instruction_nr].dest = atoi(strtok(par1, "RF"));
			instr_memory[instruction_nr].immediate = strtoul(strtok(par2, "()"), NULL, 0);
			instr_memory[instruction_nr].src1 = atoi(strtok(NULL, "R"));
			break;
		case SW:
		case SWS:
			par1 = strtok (NULL, " \t");
			par2 = strtok (NULL, " \t");
			instr_memory[instruction_nr].src1 = atoi(strtok(par1, "RF"));
			instr_memory[instruction_nr].immediate = strtoul(strtok(par2, "()"), NULL, 0);
			instr_memory[instruction_nr].src2 = atoi(strtok(NULL, "R"));
			break;
		case BEQZ:
		case BNEZ:
		case BLTZ:
		case BGTZ:
		case BLEZ:
		case BGEZ:
			par1 = strtok (NULL, " \t");
			par2 = strtok (NULL, " \t");
			instr_memory[instruction_nr].src1 = atoi(strtok(par1, "R"));
			instr_memory[instruction_nr].label = par2;
			break;
		case JUMP:
			par2 = strtok (NULL, " \t");
			instr_memory[instruction_nr].label = par2;
		default:
			break;

	} 

	/* increment instruction number before moving to next line */
	instruction_nr++;
   }
   //reconstructing the labels of the branch operations
   int i = 0;
   while(true){
   	instruction_t instr = instr_memory[i];
	if (instr.opcode == EOP) break;
	if (instr.opcode == BLTZ || instr.opcode == BNEZ ||
            instr.opcode == BGTZ || instr.opcode == BEQZ ||
            instr.opcode == BGEZ || instr.opcode == BLEZ ||
            instr.opcode == JUMP
	 ){
		instr_memory[i].immediate = (labels[instr.label] - i - 1) << 2;
	}
        i++;
   }

}

/* ============================================================================

   Simulator creation, initialization and deallocation 

   =========================================================================== */

sim_ooo::sim_ooo(unsigned mem_size,
                unsigned rob_size,
                unsigned num_int_res_stations,
                unsigned num_add_res_stations,
                unsigned num_mul_res_stations,
                unsigned num_load_res_stations,
		unsigned max_issue){
    currSim = this;
	//memory
	data_memory_size = mem_size;
    mDataMemSize = data_memory_size;
	data_memory = new unsigned char[data_memory_size];

	//issue width
	issue_width = max_issue;

	//rob, instruction window, reservation stations
    rob = new ROB(rob_size);
	//rob->num_entries=rob_size;
	pending_instructions.num_entries=rob_size;
    reservation_stations = new Reservation_Stations(num_int_res_stations,num_load_res_stations,num_add_res_stations,num_mul_res_stations);
    for(int i=0; i<reservation_stations->num_entries;i++)
        clean_res_station(&reservation_stations->entries[i]);
    //reservation_stations->num_entries= num_int_res_stations+num_load_res_stations+num_add_res_stations+num_mul_res_stations;
	//rob->entries = new rob_entry_t[rob_size];

	pending_instructions.entries = new instr_window_entry_t[rob_size];
    for(int i=0; i<pending_instructions.num_entries;i++)
        clean_instr_window(&pending_instructions.entries[i]);

	//execution units
	num_units = 0;

    for(int i=0;i<NUM_GP_REGISTERS;i++)
    {
        int_reg_file[i].val = UNDEFINED;
        int_reg_file[i].tag = UNDEFINED;
        fp_reg_file[i].val = UNDEFINED;
        fp_reg_file[i].tag = UNDEFINED;
    }

	reset();
}
	
sim_ooo::~sim_ooo(){
	delete [] data_memory;
	//delete [] rob->entries;
    delete rob;
	delete [] pending_instructions.entries;
	delete reservation_stations;
}

/* =============================================================

   CODE TO BE COMPLETED

   ============================================================= */

/* core of the simulator */
void sim_ooo::run(unsigned cycles){
    unsigned j=0u;
    while(((j<cycles) || ((cycles == 0u))) )//&& (isValidPC(PC)))// &&  && (instr_memory[PC].opcode != EOP)) && (!rob->isEmpty()))){
    {
        if((instr_memory[(PC-instr_base_address)/4].opcode == EOP) && (rob->isEmpty()))
        {
            this->clock_cycles = currClkCycle;
            break;
        }
        sim_Commit_Handler(this);
        sim_WB_Handler(this);
        sim_Exe_Handler(this);
        sim_Issue_Handler(this);
        j++;
        currClkCycle++;
    }
}

//reset the state of the simulator - please complete
void sim_ooo::reset(){

	//init instruction log
	init_log();	

	// data memory
	for (unsigned i=0; i<data_memory_size; i++) data_memory[i]=0xFF;
	
	//instr memory
	for (int i=0; i<PROGRAM_SIZE;i++){
		instr_memory[i].opcode=(opcode_t)EOP;
		instr_memory[i].src1=UNDEFINED;
		instr_memory[i].src2=UNDEFINED;
		instr_memory[i].dest=UNDEFINED;
		instr_memory[i].immediate=UNDEFINED;
	}

	//general purpose registers

	//pending_instructions

	//rob

	//reservation_stations

	//execution statistics
	clock_cycles = 0;
	instructions_executed = 0;

	//other required initializations
}

/* registers related */

int sim_ooo::get_int_register(unsigned reg){
	return int_reg_file[reg].val;; //please modify
}

void sim_ooo::set_int_register(unsigned reg, int value){
    int_reg_file[reg].val = value;
}

float sim_ooo::get_fp_register(unsigned reg){
    return unsigned2float(fp_reg_file[reg].val);
}

void sim_ooo::set_fp_register(unsigned reg, float value){
    fp_reg_file[reg].val = float2unsigned(value);
}

unsigned sim_ooo::get_int_register_tag(unsigned reg){
	return int_reg_file[reg].tag;
}

unsigned sim_ooo::get_fp_register_tag(unsigned reg){
	return fp_reg_file[reg].tag; //please modify
}

void sim_ooo::CDB_write(unsigned int tag, unsigned int val) {
    if(tag < rob->num_entries)
    {
        rob->update_dest_val(tag,val);
        reservation_stations->updateTagVal(tag,val);
    }else
    {
        //std::cout << "\n//TODO: error handling in valid tag on CDB bus";
    }
}

/*Functions of ROB*/
ROB::ROB(unsigned int mEntries) {
    num_entries = mEntries;
    if(num_entries > 0) {
        mROBTotalEntries = num_entries;
        entries = new rob_entry_t[num_entries];

        for(int i=0;i<num_entries;i++) {
            clean_rob(&entries[i]);
            entries[i].isAvailable = true;
        }
        headIndex = 0;
        tailIndex = 0;
        currLength = 0;
    }else{
        //std::cout << "\n//TODO:error handling invalid num entries init failure";
    }
}

ROB::~ROB(){
    delete [] entries;
}

bool ROB::push(unsigned mPC) {
    bool mRetVal = false;
    if((currLength < num_entries) && (isValidPC(mPC)) && (entries[tailIndex].pc == UNDEFINED) && (entries[tailIndex].isAvailable == true))
    {
        entries[tailIndex].isAvailable = false;
        entries[tailIndex].entry_instr = mInstrMemPtr[(mPC - mBaseAddr)/4];
        entries[tailIndex].pc = mPC;
        entries[tailIndex].ready = false;
        entries[tailIndex].destination = entries[tailIndex].entry_instr.dest;
        entries[tailIndex].isAddressComputed = false;

        currSim->pending_instructions.entries[tailIndex].pc = mPC;
        currSim->pending_instructions.entries[tailIndex].issue = UNDEFINED;
        currSim->pending_instructions.entries[tailIndex].exe = UNDEFINED;
        currSim->pending_instructions.entries[tailIndex].wr = UNDEFINED;
        currSim->pending_instructions.entries[tailIndex].commit = UNDEFINED;
        entries[tailIndex].state = ISSUE;
        tailIndex = (tailIndex + 1)%num_entries;  //circular buffer
        currLength++;
        mRetVal = true;
    }else
    {
        ////std::cout << "\n//TODO: error handling";
    }
    return mRetVal;
}

rob_entry_t *ROB::fetch_head() {
    rob_entry_t *mRetVal = NULL;
    if(currLength > 0)
    {
        mRetVal = &entries[headIndex];
    }
    return mRetVal;
}

bool ROB::pop() {
    bool mRetVal = false;
    if(currLength > 0)
    {
        clean_rob(&entries[headIndex]);
        clean_instr_window(&currSim->pending_instructions.entries[headIndex]);
        headIndex = (headIndex + 1)%num_entries;
        currLength--;
        mRetVal = true;
    }
    return mRetVal;
}

bool ROB::isFull(){
    return (currLength >= num_entries);
}

bool ROB::isEmpty(){
    return (currLength == 0);
}

bool ROB::update_dest_val(unsigned int entry, unsigned int val) {
    bool mRetVal = false;
    if(entry < num_entries)
    {
        entries[entry].value = val;
        entries[entry].ready = true;
    }else
    {
        //std::cout << "\n//TODO: error handling invalid ROB entry";
    }
    return mRetVal;
}

unsigned int ROB::get_entry_num(unsigned int mPC) {
    unsigned mRetVal = UNDEFINED;

    if(isValidPC(mPC)) {
        for (int i = 0; i < num_entries; i++) {
            if (entries[i].pc == mPC) {
                mRetVal = i;
                break;
            }
        }
    }
    return mRetVal;
}

unsigned int ROB::get_head_index() {
    return headIndex;
}

unsigned int ROB::get_tail_index() {
    return tailIndex;
}

/*Functions of Reservation Station*/

Reservation_Stations::Reservation_Stations(unsigned  mNum_int_res_stations, unsigned mNum_load_res_stations,
                                           unsigned mNum_add_res_stations, unsigned mNum_mul_res_stations) {
    unsigned n=0;
    num_int_stations = mNum_int_res_stations;
    num_load_stations = mNum_load_res_stations;
    num_add_stations = mNum_add_res_stations;
    num_mul_stations = mNum_mul_res_stations;
    num_entries = (num_int_stations+num_load_stations+num_add_stations+num_mul_stations);
   entries = new res_station_entry_t[num_entries];

    for (unsigned i=0; i<num_int_stations; i++,n++){
        entries[n].pc=UNDEFINED;
        entries[n].type=INTEGER_RS;
        entries[n].name=i;
        entries[n].isAvailable = true;
    }
    for (unsigned i=0; i<num_load_stations; i++,n++){
        entries[n].pc=UNDEFINED;
        entries[n].type=LOAD_B;
        entries[n].name=i;
        entries[n].isAvailable = true;
    }
    for (unsigned i=0; i<num_add_stations; i++,n++){
        entries[n].pc=UNDEFINED;
        entries[n].type=ADD_RS;
        entries[n].name=i;
        entries[n].isAvailable = true;
    }
    for (unsigned i=0; i<num_mul_stations; i++,n++){
        entries[n].pc=UNDEFINED;
        entries[n].type=MULT_RS;
        entries[n].name=i;
        entries[n].isAvailable = true;
    }
}

Reservation_Stations::~Reservation_Stations() {
    delete [] entries;
}

bool Reservation_Stations::isReservationStationAvailable(opcode_t opcode)
{
    bool mRetVal=false;
    res_station_t requiredStationType;
    requiredStationType = get_unit_type(opcode);
    for(int i=0;i<num_entries;i++)
    {
        if(entries[i].type == requiredStationType)
        {
            if((entries[i].pc == UNDEFINED) && (entries[i].isAvailable == true))
            {
                mRetVal = true;
                break;
            }
        }
    }
    return mRetVal;
}

res_station_entry_t * Reservation_Stations::fetchReservationStation(opcode_t opcode)
{
    res_station_entry_t * mRetVal= NULL;
    res_station_t requiredStationType;
    requiredStationType = get_unit_type(opcode);
    for(int i=0;i<num_entries;i++)
    {
        if(entries[i].type == requiredStationType)
        {
            if((entries[i].pc == UNDEFINED) && (entries[i].isAvailable == true))
            {
                mRetVal = &entries[i];
                break;
            }
        }
    }
    return mRetVal;
}
//Note: Insert row in reservation station after pushing the instruction to ROB
bool Reservation_Stations::insertEntry(unsigned mPC) {
    bool mRetVal = true;
    if(isValidPC(mPC))
    {
        instruction_t tempInstr = mInstrMemPtr[(mPC - mBaseAddr) / 4];

        res_station_entry_t  * tempStation = fetchReservationStation(tempInstr.opcode);
        if(tempStation)
        {
            tempStation->isAvailable = false;
            tempStation->pc = mPC;
            tempStation->entry_instr = tempInstr;
            tempStation->destination = currSim->rob->get_entry_num(mPC);
            if((tempInstr.opcode == SW) || (tempInstr.opcode == SWS))
            {
                /*SWS F1     4   (R1)           SW  R5     4   (R1)
                 *     |     |     |                 |     |     |
                 *   value1  |     |               value1  |     |
                 *          addr   |                      addr   |
                 *                value2                        value2
                 * */
                unsigned tempIndex;
                if(tempInstr.opcode == SW)
                {
                    tempIndex = tempInstr.src1;
                    if (tempIndex < NUM_GP_REGISTERS) {
                        if (currSim->int_reg_file[tempIndex].tag == UNDEFINED) {
                            /*The register is available in REG file, fetch the val and store*/
                            tempStation->value1 = currSim->int_reg_file[tempIndex].val;
                            tempStation->tag1 = UNDEFINED;
                        } else {
                            if (currSim->int_reg_file[tempIndex].tag < currSim->rob->num_entries) {
                                if (currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].ready) {
                                    /*The register is available in ROB, fetch the val and store*/
                                    tempStation->value1 = currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].value;
                                    tempStation->tag1 = UNDEFINED;
                                } else {
                                    /*The register is NOT available mark tag*/
                                    tempStation->value1 = UNDEFINED;
                                    tempStation->tag1 = currSim->int_reg_file[tempIndex].tag;
                                }
                            } else {
                                //std::cout << "\n//TODO: error handling invalid tag";
                                mRetVal = false;
                            }
                        }
                    } else {
                        //std::cout << "\n//TODO: error handling invalid src register";
                        mRetVal = false;
                    }
                }else
                {
                    tempIndex = tempInstr.src1;
                    if (tempIndex < NUM_GP_REGISTERS) {
                        if (currSim->fp_reg_file[tempIndex].tag == UNDEFINED) {
                            /*The register is available in REG file, fetch the val and store*/
                            tempStation->value1 = currSim->fp_reg_file[tempIndex].val;
                            tempStation->tag1 = UNDEFINED;
                        } else {
                            if (currSim->fp_reg_file[tempIndex].tag < currSim->rob->num_entries) {
                                if (currSim->rob->entries[currSim->fp_reg_file[tempIndex].tag].ready) {
                                    /*The register is available in ROB, fetch the val and store*/
                                    tempStation->value1 = currSim->rob->entries[currSim->fp_reg_file[tempIndex].tag].value;
                                    tempStation->tag1 = UNDEFINED;
                                } else {
                                    /*The register is NOT available mark tag*/
                                    tempStation->value1 = UNDEFINED;
                                    tempStation->tag1 = currSim->fp_reg_file[tempIndex].tag;
                                }
                            } else {
                                //std::cout << "\n//TODO: error handling invalid tag";
                                mRetVal = false;
                            }
                        }
                    } else {
                        //std::cout << "\n//TODO: error handling invalid src register";
                        mRetVal = false;
                    }
                }

                tempIndex = tempInstr.src2;
                if(tempIndex< NUM_GP_REGISTERS)
                {
                    if(currSim->int_reg_file[tempIndex].tag == UNDEFINED)
                    {
                        /*The register is available, fetch the val and store*/
                        tempStation->value2 = currSim->int_reg_file[tempIndex].val;
                        tempStation->tag2 = UNDEFINED;
                    }else
                    {
                        if(currSim->int_reg_file[tempIndex].tag < currSim->rob->num_entries)
                        {
                            if(currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].ready)
                            {
                                /*The register is available in ROB, fetch the val and store*/
                                tempStation->value2 = currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].value;
                                tempStation->tag2 = UNDEFINED;
                            }else
                            {
                                /*The register is NOT available mark tag*/
                                tempStation->value2 = UNDEFINED;
                                tempStation->tag2 = currSim->int_reg_file[tempIndex].tag;
                            }
                        } else
                        {
                            //std::cout << "\n//TODO: error handling invalid tag";
                            mRetVal = false;
                        }
                    }
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid src register";
                    mRetVal = false;
                }

                tempStation->address = tempInstr.immediate;


            }else if(tempInstr.opcode == JUMP)
            {
                /*JUMP T
                 *     |
                 *   value1
                 *
                 * */
                tempStation->value1 = tempInstr.immediate;
                tempStation->tag1 = UNDEFINED;
                tempStation->value2 = 0;
                tempStation->tag2 = UNDEFINED;

            }else if((is_branch(tempInstr.opcode)) || (tempInstr.opcode == LW) ||
                    ((tempInstr.opcode == LWS)) || (is_int_imm(tempInstr.opcode)))
            {
                /*BEQZ R1    T           LW(S)  R3   20   (R2)      ADDI R2   R0    0x4
                 *     |     |            *          |     |        *          |     |
                 *   value1  |            *       value2   |        *       value1   |
                 *          value2        *             value1      *             value2
                 * */
                unsigned tempIndex;
                tempIndex = tempInstr.src1;
                if(tempIndex< NUM_GP_REGISTERS)
                {
                    if(currSim->int_reg_file[tempIndex].tag == UNDEFINED)
                    {
                        /*The register is available in REG file, fetch the val and store*/
                        tempStation->value1 = currSim->int_reg_file[tempIndex].val;
                        tempStation->tag1 = UNDEFINED;
                    }else
                    {
                        if(currSim->int_reg_file[tempIndex].tag < currSim->rob->num_entries)
                        {
                            if(currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].ready)
                            {
                                /*The register is available in ROB, fetch the val and store*/
                                tempStation->value1 = currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].value;
                                tempStation->tag1 = UNDEFINED;
                            }else
                            {
                                /*The register is NOT available mark tag*/
                                tempStation->value1 = UNDEFINED;
                                tempStation->tag1 = currSim->int_reg_file[tempIndex].tag;
                            }
                        } else
                        {
                            //std::cout << "\n//TODO: error handling invalid tag";
                            mRetVal = false;
                        }
                    }
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid src register";
                    mRetVal = false;
                }
                if((tempInstr.opcode == LW) || ((tempInstr.opcode == LWS)))
                {
                    tempStation->value2 = UNDEFINED;
                    tempStation->address = tempInstr.immediate;
                    tempStation->tag2 = UNDEFINED;
                }else
                {
                    tempStation->value2 = tempInstr.immediate;
                    tempStation->tag2 = UNDEFINED;
                }
                if((is_branch(tempInstr.opcode)) || (is_int_imm(tempInstr.opcode)))
                {
                    tempStation->value2 = UNDEFINED;
                    tempStation->tag2 = UNDEFINED;
                }
                if(!(is_branch(tempInstr.opcode))) {
                    //Update destination register tags
                    tempIndex = tempInstr.dest;
                    if (tempIndex < NUM_GP_REGISTERS) {
                        if (((tempInstr.opcode == LW)) || (is_int_imm(tempInstr.opcode))) {
                            currSim->int_reg_file[tempIndex].tag = currSim->rob->get_entry_num(mPC);
                        } else if (tempInstr.opcode == LWS) {
                            currSim->fp_reg_file[tempIndex].tag = currSim->rob->get_entry_num(mPC);
                        } else {
                            //std::cout << "\n//TODO: error handling invalid destination opcode";
                            mRetVal = false;
                        }
                    } else {
                        //std::cout << "\n//TODO: error handling invalid destination";
                        mRetVal = false;
                    }
                }

            }else if(is_int_r(tempInstr.opcode))
            {
                /*ADD  R3   R0   R2
                 *          |     |
                 *       value1   |
                 *             value2
                 * */
                unsigned tempIndex;
                tempIndex = tempInstr.src1;
                if(tempIndex< NUM_GP_REGISTERS)
                {
                    if(currSim->int_reg_file[tempIndex].tag == UNDEFINED)
                    {
                        /*The register is available, fetch the val and store*/
                        tempStation->value1 = currSim->int_reg_file[tempIndex].val;
                        tempStation->tag1 = UNDEFINED;
                    }else
                    {
                        if(currSim->int_reg_file[tempIndex].tag < currSim->rob->num_entries)
                        {
                            if(currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].ready)
                            {
                                /*The register is available in ROB, fetch the val and store*/
                                tempStation->value1 = currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].value;
                                tempStation->tag1 = UNDEFINED;
                            }else
                            {
                                /*The register is NOT available mark tag*/
                                tempStation->value1 = UNDEFINED;
                                tempStation->tag1 = currSim->int_reg_file[tempIndex].tag;
                            }
                        } else
                        {
                            //std::cout << "\n//TODO: error handling invalid tag";
                            mRetVal = false;
                        }
                    }
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid src register";
                    mRetVal = false;
                }

                tempIndex = tempInstr.src2;
                if(tempIndex< NUM_GP_REGISTERS)
                {
                    if(currSim->int_reg_file[tempIndex].tag == UNDEFINED)
                    {
                        /*The register is available, fetch the val and store*/
                        tempStation->value2 = currSim->int_reg_file[tempIndex].val;
                        tempStation->tag2 = UNDEFINED;
                    }else
                    {
                        if(currSim->int_reg_file[tempIndex].tag < currSim->rob->num_entries)
                        {
                            if(currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].ready)
                            {
                                /*The register is available in ROB, fetch the val and store*/
                                tempStation->value2 = currSim->rob->entries[currSim->int_reg_file[tempIndex].tag].value;
                                tempStation->tag2 = UNDEFINED;
                            }else
                            {
                                /*The register is NOT available mark tag*/
                                tempStation->value2 = UNDEFINED;
                                tempStation->tag2 = currSim->int_reg_file[tempIndex].tag;
                            }
                        } else
                        {
                            //std::cout << "\n//TODO: error handling invalid tag";
                            mRetVal = false;
                        }
                    }
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid src register";
                    mRetVal = false;
                }

                //Update destination register tags
                tempIndex = tempInstr.dest;
                if(tempIndex < NUM_GP_REGISTERS)
                {
                    currSim->int_reg_file[tempIndex].tag = currSim->rob->get_entry_num(mPC);
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid destination";
                    mRetVal = false;
                }

            }else if(is_fp_alu(tempInstr.opcode))
            {
                /*SUBS F2  F5     F4
                 *          |     |
                 *       value1   |
                 *             value2
                 * */
                unsigned tempIndex;
                tempIndex = tempInstr.src1;
                if(tempIndex< NUM_GP_REGISTERS)
                {
                    if(currSim->fp_reg_file[tempIndex].tag == UNDEFINED)
                    {
                        /*The register is available, fetch the val and store*/
                        tempStation->value1 = currSim->fp_reg_file[tempIndex].val;
                        tempStation->tag1 = UNDEFINED;
                    }else
                    {
                        if(currSim->fp_reg_file[tempIndex].tag < currSim->rob->num_entries)
                        {
                            if(currSim->rob->entries[currSim->fp_reg_file[tempIndex].tag].ready)
                            {
                                /*The register is available in ROB, fetch the val and store*/
                                tempStation->value1 = currSim->rob->entries[currSim->fp_reg_file[tempIndex].tag].value;
                                tempStation->tag1 = UNDEFINED;
                            }else
                            {
                                /*The register is NOT available mark tag*/
                                tempStation->value1 = UNDEFINED;
                                tempStation->tag1 = currSim->fp_reg_file[tempIndex].tag;
                            }
                        } else
                        {
                            //std::cout << "\n//TODO: error handling invalid tag";
                            mRetVal = false;
                        }
                    }
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid src register";
                    mRetVal = false;
                }

                tempIndex = tempInstr.src2;
                if(tempIndex< NUM_GP_REGISTERS)
                {
                    if(currSim->fp_reg_file[tempIndex].tag == UNDEFINED)
                    {
                        /*The register is available, fetch the val and store*/
                        tempStation->value2 = currSim->fp_reg_file[tempIndex].val;
                        tempStation->tag2 = UNDEFINED;
                    }else
                    {
                        if(currSim->fp_reg_file[tempIndex].tag < currSim->rob->num_entries)
                        {
                            if(currSim->rob->entries[currSim->fp_reg_file[tempIndex].tag].ready)
                            {
                                /*The register is available in ROB, fetch the val and store*/
                                tempStation->value2 = currSim->rob->entries[currSim->fp_reg_file[tempIndex].tag].value;
                                tempStation->tag2 = UNDEFINED;
                                mRetVal = true;
                            }else
                            {
                                /*The register is NOT available mark tag*/
                                tempStation->value2 = UNDEFINED;
                                tempStation->tag2 = currSim->fp_reg_file[tempIndex].tag;
                                mRetVal = true;
                            }
                        } else
                        {
                            //std::cout << "\n//TODO: error handling invalid tag";
                            mRetVal = false;
                        }
                    }
                } else
                {
                    //std::cout << "\n//TODO: error handling invalid src register";
                    mRetVal = false;
                }
                //Update destination register tags
                tempIndex = tempInstr.dest;
                if(tempIndex < NUM_GP_REGISTERS)
                {
                    currSim->fp_reg_file[tempIndex].tag = currSim->rob->get_entry_num(mPC);
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid destination";
                    mRetVal = false;
                }

            }else
            {
                //std::cout << "\n//TODO: error handling invalid opcode reservation station insert";
                mRetVal = false;
            }

            //undo if could NOT insert
            if(mRetVal == false)
            {
                tempStation->pc = UNDEFINED;
            }

        } else if((tempInstr.opcode == SW) || (tempInstr.opcode == SWS))
        {
            //Do Nothing SW/SWS do NOT need reservation station
            mRetVal = false;
        }else
        {
            //std::cout << "\n//TODO: could NOT fetch reservation station";
            mRetVal = false;
        }
    }else
    {
        //std::cout << "\n//TODO: error handling invalid PC";
    }
    return  mRetVal;
}

void Reservation_Stations::updateTagVal(unsigned int tag, unsigned int val) {
    for(int i=0;i<num_entries;i++)
    {
        if(entries[i].tag1 == tag)
        {
            entries[i].tag1 = UNDEFINED;
            entries[i].value1 = val;
            entries[i].CDBWriteDataAvailClkCycle = currSim->pending_instructions.entries[tag].wr;
        }
        if(entries[i].tag2 == tag)
        {
            entries[i].tag2 = UNDEFINED;
            entries[i].value2 = val;
            entries[i].CDBWriteDataAvailClkCycle = currSim->pending_instructions.entries[tag].wr;
            if(isStoreInstr(entries[i].entry_instr.opcode))
            {
                entries[i].CDBWriteDataAvailClkCyclevalue2 = entries[i].CDBWriteDataAvailClkCycle;
            }
        }
    }
}
res_station_t Reservation_Stations::get_unit_type(opcode_t opcode) {

    res_station_t mRetVal = MAX_RS;
        switch(opcode){
            //Integer unit
            case ADD:
            case ADDI:
            case SUB:
            case SUBI:
            case XOR:
            case AND:
            case BEQZ:
            case BNEZ:
            case BLTZ:
            case BGTZ:
            case BLEZ:
            case BGEZ:
            case JUMP:
                mRetVal = INTEGER_RS;
                break;
                //memory unit
            case SW:
            case SWS:
/*                mRetVal = MAX_RS;
                //No need reservation stations for Store
                break;*/
            case LW:
            case LWS:
                mRetVal = LOAD_B;
                break;
                // FP adder
            case ADDS:
            case SUBS:
                mRetVal = ADD_RS;
                break;
                // Multiplier
            case MULT:
            case MULTS:
            case DIV:
            case DIVS:
                mRetVal = MULT_RS;
                break;
            default:
                mRetVal = MAX_RS;
                //std::cout << "\n//TODO: error handling invalid opcode";
                break;
        }
    return mRetVal;
}

unsigned int Reservation_Stations::get_station_num(unsigned int mPC) {
    unsigned mRetVal = UNDEFINED;
    for(int i=0;i<num_entries;i++)
    {
        if(entries[i].pc == mPC)
        {
            mRetVal = i;
            break;
        }
    }
    return mRetVal;
}
bool isOpcodeFpType(opcode_t mOpCode)
{
    bool mRetVal=false;
    switch (mOpCode) {
        case ADD:
        case ADDI:
        case SUB:
        case SUBI:
        case XOR:
        case AND:
        case BEQZ:
        case BNEZ:
        case BLTZ:
        case BGTZ:
        case BLEZ:
        case BGEZ:
        case JUMP:
        case LW:
        case SW:
        case DIV:
        case MULT:
            mRetVal=false;
            break;
        case LWS:
        case SWS:
        case ADDS:
        case SUBS:
        case MULTS:
        case DIVS:
            mRetVal=true;
            break;
        default:
            //std::cout << "\n//TODO:error handling invalid opcode";
            mRetVal= false;
            break;

    }
    return mRetVal;
}

void sim_Issue_Handler(sim_ooo * mSim)
{
    //check if reservation station and ROB are available
    instruction_t currInstr;
    if(!isBranchMispredicted) {
        for (int i = 0; i < mSim->issue_width; i++) {
            if (isValidPC(mSim->PC)) {
                currInstr = mSim->instr_memory[(mSim->PC - mBaseAddr) / 4];
                //check if ROB is available
                if (currInstr.opcode != EOP) {
                    if (!mSim->rob->isFull()) {
                        if (mSim->reservation_stations->isReservationStationAvailable(currInstr.opcode)) {
                            if (mSim->rob->push(mSim->PC)) {
                                //ROB push success
                                if (mSim->reservation_stations->insertEntry(mSim->PC)) {
                                    //reservation station insert success
                                    //increment program counter
                                    update_instr_window(mSim->PC, ISSUE);
                                    mSim->PC += 4;
                                } else {
                                    //std::cout << "\n//TODO: error handling reservation station insert failure";
                                }
                            } else {
                                //std::cout << "\n//TODO: error handling ROB push failure (shouldn't hit this line coz full check done)";
                            }
                        } else {
                            //reservation station not available
                            break;
                        }
                    } else {
                        //ROB full
                        break;
                    }
                } else {
                    //reached EOP
                    break;
                }
            } else {
                //std::cout << "\n//TODO: error handling invalid PC at issue";
            }
        }
    }else
    {
        isBranchMispredicted = false;
    }
}
void sim_Exe_Handler(sim_ooo * mSim)
{
    //check each entry of reservation station. if operands available and required exec unit available then send the instr to its exec unit
    for(int i=0;i<mSim->reservation_stations->num_entries;i++)
    {
        res_station_entry_t * currStationEntry;
        currStationEntry = &mSim->reservation_stations->entries[i];

        //check if the instruction is already being executed
        if((isValidPC(currStationEntry->pc)) && (search_exe_unit(currStationEntry->pc) == UNDEFINED))
        {
                //check if the operands are available
                if ((currStationEntry->tag1 == UNDEFINED) && (currStationEntry->tag2 == UNDEFINED))
                {
                    //check if there's a pending store to the same address if the instr is a load
                    if((!isLoadInstr(currStationEntry->entry_instr.opcode)) ||
                       ((is_memory(currStationEntry->entry_instr.opcode)) && (search_prev_load_store(currStationEntry) == UNDEFINED))) {
                        //All the required operands are available send the instruction to corresponding execution unit if available

                        if (currStationEntry->CDBWriteDataAvailClkCycle < (int)currClkCycle)
                        {
                            if(((isLoadInstr(currStationEntry->entry_instr.opcode) && (currStationEntry->value2 != UNDEFINED)) || (currStationEntry->entry_instr.opcode == SW) || (currStationEntry->entry_instr.opcode == SWS)) &&
                               (currSim->rob->entries[currSim->rob->get_entry_num(currStationEntry->pc)].isAddressComputed == false))
                            {
                                if((mDummyExeUnit[mCurrDummyUnitIndex].pc == UNDEFINED) && (mDummyExeUnit[mCurrDummyUnitIndex].isAvailable == true))
                                {
                                    currSim->rob->entries[currSim->rob->get_entry_num(currStationEntry->pc)].isAddressComputed = true;
                                    if(isLoadInstr(currStationEntry->entry_instr.opcode))
                                    {
                                        mDummyExeUnit[mCurrDummyUnitIndex].pc = currStationEntry->pc;
                                        mDummyExeUnit[mCurrDummyUnitIndex].unit_instr = currStationEntry->entry_instr;
                                        mDummyExeUnit[mCurrDummyUnitIndex].isAvailable = false;
                                        //store bypassing done for this load
                                        //if(currStationEntry->value2 != UNDEFINED)
                                        //{
                                            mDummyExeUnit[mCurrDummyUnitIndex].output = currStationEntry->value2;
                                            update_instr_window(currStationEntry->pc, EXECUTE);
                                            currSim->rob->entries[currSim->rob->get_entry_num(currStationEntry->pc)].state = EXECUTE;
                                        //}
                                        mDummyExeUnit[mCurrDummyUnitIndex].reservationStationIndex = i;
                                        currStationEntry->address = currStationEntry->value1 + currStationEntry->address;
                                    }else
                                    {
                                        mDummyExeUnit[mCurrDummyUnitIndex].pc = currStationEntry->pc;
                                        mDummyExeUnit[mCurrDummyUnitIndex].unit_instr = currStationEntry->entry_instr;
                                        mDummyExeUnit[mCurrDummyUnitIndex].isAvailable = false;
                                        mDummyExeUnit[mCurrDummyUnitIndex].output = currStationEntry->value2;
                                        mDummyExeUnit[mCurrDummyUnitIndex].reservationStationIndex = i;
                                        currStationEntry->address = currStationEntry->value2 + currStationEntry->address;
                                        currSim->rob->entries[currSim->rob->get_entry_num(mDummyExeUnit[mCurrDummyUnitIndex].pc)].destination = currSim->reservation_stations->entries[mDummyExeUnit[mCurrDummyUnitIndex].reservationStationIndex].address;
                                        mDummyExeUnit[mCurrDummyUnitIndex].output = currSim->reservation_stations->entries[mDummyExeUnit[mCurrDummyUnitIndex].reservationStationIndex].value1;
                                        update_instr_window(currStationEntry->pc, EXECUTE);
                                        currSim->rob->entries[currSim->rob->get_entry_num(currStationEntry->pc)].state = EXECUTE;
                                    }
                                    mCurrDummyUnitIndex = (mCurrDummyUnitIndex + 1)%mNumDummyUnits;
                                }
                                continue;
                            }
                            unsigned tempUnitIndex = mSim->get_free_unit(currStationEntry->entry_instr.opcode);
                            if (tempUnitIndex != UNDEFINED)
                            {
                                //required unit is available pass the station data to the unit
                                mSim->exec_units[tempUnitIndex].pc = currStationEntry->pc;
                                mSim->exec_units[tempUnitIndex].unit_instr = currStationEntry->entry_instr;
                                mSim->exec_units[tempUnitIndex].busy = mSim->exec_units[tempUnitIndex].latency;
                                mSim->exec_units[tempUnitIndex].reservationStationIndex = i;
                                mSim->exec_units[tempUnitIndex].isAvailable = false;
                                if (isLoadInstr(currStationEntry->entry_instr.opcode))
                                {
                                    currStationEntry->address = currStationEntry->value1 + currStationEntry->address;
                                }
                                update_instr_window(currStationEntry->pc, EXECUTE);
                                currSim->rob->entries[currSim->rob->get_entry_num(currStationEntry->pc)].state = EXECUTE;

                            } else
                            {
                                //exec unit not yet available wait and retry next cycle
                            }
                        } else
                        {
                            //currStationEntry->CDBWriteDataAvailClkCycle = UNDEFINED;
                        }
                    }
                } else {
                    //operands not yet available wait and retry next cycle
                }
        }
    }

    //Process all exec units
    for(int i=0; i<mSim->num_units; i++)
    {
        unit_t * currUnit = &mSim->exec_units[i];
        if((currUnit->pc != UNDEFINED) && (currUnit->busy > 0))
        {
            currUnit->busy--;
        }else
        {
            //Unit is empty nothing to do;
        }
    }

    for(int i=0; i<mSim->num_units; i++)
    {
        unit_t * currUnit = &mSim->exec_units[i];
        unsigned tempDataMemAddr;
        /*if((currUnit->pc != UNDEFINED) && (currUnit->busy == (currUnit->latency-1)) && (isLoadInstr(currUnit->unit_instr.opcode)))
        {
            tempDataMemAddr = mSim->reservation_stations->entries[currUnit->reservationStationIndex].value1 +
                              mSim->reservation_stations->entries[currUnit->reservationStationIndex].address;
            mSim->reservation_stations->entries[currUnit->reservationStationIndex].address = tempDataMemAddr;
        }*/
        if((currUnit->pc != UNDEFINED) && (currUnit->busy == 0) /*&& (mSim->rob->entries[mSim->rob->get_entry_num(currUnit->pc)].state == EXECUTE)*/)
        {
            if(currUnit->type == MEMORY)
            {
                if((currUnit->unit_instr.opcode == LW) || (currUnit->unit_instr.opcode == LWS))
                {
                    //check if store bypassed for this load by checking if value2 is undefined or not
                    if(mSim->reservation_stations->entries[currUnit->reservationStationIndex].value2 == UNDEFINED)
                    {
                        tempDataMemAddr = mSim->reservation_stations->entries[currUnit->reservationStationIndex].address;
                        if (tempDataMemAddr < mSim->data_memory_size) {
                            currUnit->output = char2unsigned(&mSim->data_memory[tempDataMemAddr]);
                        } else {
                            //std::cout << "\n//TODO: error handling invalid data mem address";
                        }
                    } else
                    {
                        currUnit->output = mSim->reservation_stations->entries[currUnit->reservationStationIndex].value2;
                    }

                }else if((currUnit->unit_instr.opcode == SW) || (currUnit->unit_instr.opcode == SWS))
                {
                    unsigned regVal;
                    unsigned tempAddr;
                    //SW(S) enters exe twice once for addr and another time for actual mem access
                    //use state var to identify which access and perform the required action
                    if(currSim->rob->entries[currSim->rob->get_entry_num(currUnit->pc)].state == EXECUTE)
                    {
                        //SW first EXE access
                        /*station address = station value2 + station address(immediate val)*/
                        currSim->reservation_stations->entries[currUnit->reservationStationIndex].address = currSim->reservation_stations->entries[currUnit->reservationStationIndex].value2 + currSim->reservation_stations->entries[currUnit->reservationStationIndex].address;
                        currSim->rob->entries[currSim->rob->get_entry_num(currUnit->pc)].destination = currSim->reservation_stations->entries[currUnit->reservationStationIndex].address;
                        currUnit->output = currSim->reservation_stations->entries[currUnit->reservationStationIndex].value1;
                    }else
                    {
                        //ROB value field contains the store address
                        //SW second EXE access
                        tempAddr = mSim->rob->entries[mSim->rob->get_entry_num(currUnit->pc)].destination;
                        regVal = mSim->rob->entries[mSim->rob->get_entry_num(currUnit->pc)].value;
                        if (tempAddr < mSim->data_memory_size) {
                            unsigned2char(regVal, &mSim->data_memory[tempAddr]);
                        } else {
                            //std::cout << "\n//TODO: invalid data memory address";
                        }
                        //release
                        currUnit->pc = UNDEFINED;
                        currUnit->isAvailable = true;
                        if (!mSim->rob->pop()) {
                            //std::cout << "\n//TODO: error handling SW ROB pop failure";
                        }
                        mSim->instructions_executed++;
                    }
                }else
                {
                    //std::cout << "\n//TODO: error handling invalid opcode in MEMORY exec unit";
                }
            }else
            {
                //if exec unit is processing branch instruction then store the branch address to the output
                if(currUnit->unit_instr.opcode == JUMP)
                {
                    currUnit->output = alu(currUnit->unit_instr.opcode, UNDEFINED, UNDEFINED, mSim->reservation_stations->entries[currUnit->reservationStationIndex].value1, currUnit->pc);
                }else if(is_branch(currUnit->unit_instr.opcode))
                {
                    currUnit->output = alu(currUnit->unit_instr.opcode, mSim->reservation_stations->entries[currUnit->reservationStationIndex].value1, UNDEFINED, mSim->reservation_stations->entries[currUnit->reservationStationIndex].entry_instr.immediate, currUnit->pc);
                }else if(is_int_imm(currUnit->unit_instr.opcode))
                {
                    currUnit->output = alu(currUnit->unit_instr.opcode, mSim->reservation_stations->entries[currUnit->reservationStationIndex].value1, mSim->reservation_stations->entries[currUnit->reservationStationIndex].entry_instr.immediate, UNDEFINED,  currUnit->pc);
                }else
                {
                    currUnit->output = alu(currUnit->unit_instr.opcode, mSim->reservation_stations->entries[currUnit->reservationStationIndex].value1, mSim->reservation_stations->entries[currUnit->reservationStationIndex].value2, UNDEFINED, currUnit->pc);
                }
            }

        }else
        {
            //Unit is empty nothing to do;
        }
    }
}
void sim_WB_Handler(sim_ooo * mSim)
{
    for(int i=0; i < mSim->num_units; i++)
    {
        unit_t *currUnit = &mSim->exec_units[i];
        if (isValidPC(currUnit->pc) && (currUnit->busy == 0))
        {
            currSim->rob->entries[currSim->rob->get_entry_num(currUnit->pc)].state = WRITE_RESULT;
            update_instr_window(currUnit->pc, WRITE_RESULT);
            mSim->CDB_write(mSim->reservation_stations->entries[currUnit->reservationStationIndex].destination,currUnit->output);
            //release execution unit and reservation station entry
            currUnit->pc = UNDEFINED;
            clean_res_station(&mSim->reservation_stations->entries[currUnit->reservationStationIndex]);
        }
    }
    for(int i=0;i<mNumDummyUnits;i++) {
        if (isValidPC(mDummyExeUnit[i].pc))
        {
            res_station_entry_t *currStation;
            currStation = &mSim->reservation_stations->entries[mDummyExeUnit[i].reservationStationIndex];
            if (((isLoadInstr(mDummyExeUnit[i].unit_instr.opcode)) && (currStation->value2 != UNDEFINED)) ||
                ((mDummyExeUnit[i].unit_instr.opcode == SW) || (mDummyExeUnit[i].unit_instr.opcode == SWS))) {
                currSim->rob->entries[currSim->rob->get_entry_num(mDummyExeUnit[i].pc)].state = WRITE_RESULT;
                update_instr_window(mDummyExeUnit[i].pc, WRITE_RESULT);
                mSim->CDB_write(mSim->reservation_stations->entries[mDummyExeUnit[i].reservationStationIndex].destination,
                                mDummyExeUnit[i].output);

                //release execution unit and reservation station entry
                clean_res_station(&mSim->reservation_stations->entries[mDummyExeUnit[i].reservationStationIndex]);
            }
            mCurrDummyUnitIndex--;
            mDummyExeUnit[i].pc = UNDEFINED;
            mDummyExeUnit[i].isAvailable = true;
        }
    }
    for(int i=0; i<mSim->rob->num_entries;i++)
    {
        instruction_t currInstr = mSim->rob->entries[i].entry_instr;
        if((currInstr.opcode == SW) || (currInstr.opcode == SWS))
        {
            if(mSim->rob->entries[i].state == WRITE_RESULT)
            {
                store_bypassing_wb_handler(mSim->rob->entries[i].pc);
            }
        }
    }
}
void sim_Commit_Handler(sim_ooo * mSim)
{
    //workaround to make rob entries, reservation stations and exe units available for exe in the next clock cycle
    for(int i=0; i < mSim->num_units; i++)
    {
        unit_t *currUnit = &mSim->exec_units[i];
        if(currUnit->pc == UNDEFINED)
        {
            currUnit->isAvailable = true;
        }
    }
    for(int i=0; i < mSim->reservation_stations->num_entries; i++)
    {
        if(mSim->reservation_stations->entries[i].pc == UNDEFINED)
        {
            mSim->reservation_stations->entries[i].isAvailable = true;
        }
    }
    for(int i=0; i<mSim->rob->num_entries; i++)
    {
        if(mSim->rob->entries[i].pc == UNDEFINED)
        {
            mSim->rob->entries[i].isAvailable = true;
        }
    }
    rob_entry_t * currHead;
    for(int i=0; i < 1/*mSim->issue_width*/; i++)
    {
        currHead = mSim->rob->fetch_head();
        if(currHead)
        {
            if(currHead->pc == 0xc)
            {
                //std::cout << "\ncommit handler pc = 0xc\n";
            }
            //SW(S) enters exe twice once for addr and another time for actual mem access
            //use state var to identify which access and perform the required action
            if (((currHead->entry_instr.opcode == SW) || (currHead->entry_instr.opcode == SWS)))
            {
                if(currHead->state == WRITE_RESULT)
                {
                    //check if SW(s) is already being executed
                    if (search_exe_unit(currHead->pc) == UNDEFINED) {
                        unsigned tempExeUnitIndex;
                        ////std::cout << "\n//TODO:store handling";
                        tempExeUnitIndex = currSim->get_free_unit(currHead->entry_instr.opcode);
                        //check if required exe unit is available
                        if (tempExeUnitIndex != UNDEFINED) {
                            currSim->exec_units[tempExeUnitIndex].pc = currHead->pc;
                            currSim->exec_units[tempExeUnitIndex].unit_instr = currHead->entry_instr;
                            currSim->exec_units[tempExeUnitIndex].busy = currSim->exec_units[tempExeUnitIndex].latency;
                            currSim->exec_units[tempExeUnitIndex].reservationStationIndex = currSim->rob->get_head_index();
                            mSim->rob->entries[mSim->rob->get_entry_num(currHead->pc)].state = COMMIT;
                            update_instr_window(currHead->pc, COMMIT);
                            mSim->commit_to_log(mSim->pending_instructions.entries[mSim->rob->get_entry_num(currHead->pc)]);

                        }
                    } else {
                        //Do Nothing SW is already being committed wait...
                    }
                }
                break;
            } else if (currHead->ready)
            {
                mSim->instructions_executed++;
                update_instr_window(currHead->pc, COMMIT);
                //mSim->commit_to_log(mSim->pending_instructions.entries[mSim->rob->get_entry_num(currHead->pc)]);
                if (is_branch(currHead->entry_instr.opcode)) {
                    if (currHead->value == currHead->pc + 4) {
                        //Do Nothing branch not taken
                        mSim->commit_to_log(mSim->pending_instructions.entries[mSim->rob->get_entry_num(currHead->pc)]);
                        if (!mSim->rob->pop()) {
                            //std::cout << "\n//TODO: error handling pop failure at commit clearing buffer";
                        }
                    } else {
                        //Branch mis prediction handling
                        sim_WB_Handler(mSim);
                        sim_Exe_Handler(mSim);
                        sim_Issue_Handler(mSim);
                        int n=0;
                        for(int i=mSim->rob->get_head_index();n<mSim->rob->currLength;i=(i+1)%mSim->rob->num_entries,n++)
                        {
                            mSim->commit_to_log(mSim->pending_instructions.entries[i]);
                        }
                        isBranchMispredicted = true;
                        mSim->PC = currHead->value;
                        if (!isValidPC(mSim->PC)) {
                            //std::cout << "\n//TODO: error handling invalid PC loaded at commit";
                        }
                        //Clear ROB
                        while (!mSim->rob->isEmpty()) {
                            if (!mSim->rob->pop()) {
                                //std::cout << "\n//TODO: error handling pop failure at commit clearing buffer";
                            }
                        }
                        mSim->rob->headIndex = 0;
                        mSim->rob->tailIndex = 0;

                        for(int i=0; i<mSim->rob->num_entries; i++)
                        {
                            mSim->rob->entries[i].isAvailable = true;
                        }
                        //Clear Exec units
                        for (int i = 0; i < mSim->num_units; i++) {
                            mSim->exec_units[i].pc = UNDEFINED;
                            mSim->exec_units[i].busy = 0;
                            mSim->exec_units[i].isAvailable = true;
                        }
                        //Clear reservation stations
                        for (int i = 0; i < mSim->reservation_stations->num_entries; i++) {
                            clean_res_station(&mSim->reservation_stations->entries[i]);
                            mSim->reservation_stations->entries[i].isAvailable = true;
                        }
                        for(int i = 0;i<NUM_GP_REGISTERS;i++)
                        {
                            mSim->int_reg_file[i].tag = UNDEFINED;
                            mSim->fp_reg_file[i].tag = UNDEFINED;
                        }
                        break;
                    }

                } else if (isOpcodeFpType(currHead->entry_instr.opcode)) {
                    mSim->commit_to_log(mSim->pending_instructions.entries[mSim->rob->get_entry_num(currHead->pc)]);
                    mSim->fp_reg_file[currHead->destination].val = currHead->value;
                    //clear tag if it corresponds to the  current ROB entry
                    if (mSim->fp_reg_file[currHead->destination].tag == mSim->rob->get_entry_num(currHead->pc)) {
                        mSim->fp_reg_file[currHead->destination].tag = UNDEFINED;
                    }
                    //release rob entry
                    if (!mSim->rob->pop()) {
                        //std::cout << "\n//TODO: error handling pop failure";
                    }
                } else {
                    mSim->commit_to_log(mSim->pending_instructions.entries[mSim->rob->get_entry_num(currHead->pc)]);
                    mSim->int_reg_file[currHead->destination].val = currHead->value;
                    //clear tag if it corresponds to the  current ROB entry
                    if (mSim->int_reg_file[currHead->destination].tag == mSim->rob->get_entry_num(currHead->pc)) {
                        mSim->int_reg_file[currHead->destination].tag = UNDEFINED;
                    }
                    //release rob entry
                    if (!mSim->rob->pop()) {
                        //std::cout << "\n//TODO: error handling pop failure";
                    }
                }
            } else
            {
                //Curr Head not ready so stop and do NOT proceed to commit other instruction even issue width> 1
                break;
            }
        }else
        {
            //curr head null
            break;
        }
    }
}

inline bool isValidPC(unsigned mPC)
{
    return ((mPC >= mBaseAddr) && (mPC < (mBaseAddr + (4*PROGRAM_SIZE))));
}

inline bool isLoadInstr(opcode_t mOpCode)
{
    return ((mOpCode ==  LW) || (mOpCode == LWS));
}

inline bool isStoreInstr(opcode_t mOpCode)
{
    return ((mOpCode ==  SW) || (mOpCode == SWS));
}

unsigned search_exe_unit(unsigned mPC)
{
    unsigned mRetVal = UNDEFINED;
    if(isValidPC(mPC))
    {
        for(int i=0;i<currSim->num_units;i++)
        {
            if(currSim->exec_units[i].pc == mPC)
            {
                mRetVal = i;
                break;
            }
        }
    }

    return mRetVal;
}

unsigned search_prev_load_store(res_station_entry_t * mStation)
{
    unsigned mRetval = UNDEFINED;
    unsigned mROBIndex = currSim->rob->get_entry_num(mStation->pc);
    rob_entry_t * mROBEntry = &currSim->rob->entries[mROBIndex];
    for(int i=currSim->rob->get_head_index(); i!=mROBIndex;i=(i+1)%currSim->rob->num_entries)
    {
        rob_entry_t * currROBEntry = &currSim->rob->entries[i];
        res_station_entry_t * currStation = &currSim->reservation_stations->entries[currSim->reservation_stations->get_station_num(currROBEntry->pc)];
        if(isStoreInstr(currROBEntry->entry_instr.opcode))
        {
            //if(isLoadInstr(mStation->entry_instr.opcode))

            if (currROBEntry->state == ISSUE)
            {
                if ((currStation->tag2 == UNDEFINED) && (currStation->CDBWriteDataAvailClkCyclevalue2 < (int)currClkCycle))
                {
                    unsigned temp1Addr;
                    if(isLoadInstr(mStation->entry_instr.opcode))
                    {
                        temp1Addr = mStation->value1 + mStation->entry_instr.immediate;
                    } else if(isStoreInstr(mStation->entry_instr.opcode))
                    {
                        temp1Addr = mStation->value2 + mStation->entry_instr.immediate;
                    }
                    if ((currStation->value2 + currStation->entry_instr.immediate) == temp1Addr)
                    {
                        mRetval = i;
                        break;
                    }
                } else
                {
                    mRetval = i;
                    break;
                }
            } else if (currROBEntry->state == EXECUTE)
            {
                unsigned temp1Addr;
                if(isLoadInstr(mStation->entry_instr.opcode))
                {
                    temp1Addr = mStation->value1 + mStation->entry_instr.immediate;
                } else if(isStoreInstr(mStation->entry_instr.opcode))
                {
                    temp1Addr = mStation->value2 + mStation->entry_instr.immediate;
                }
                if ((currStation->address) == temp1Addr)
                {
                    mRetval = i;
                    break;
                }
            } else {
                //Do Nothing
            }

        }/* else if(isLoadInstr(currROBEntry->entry_instr.opcode))
        {
            if(isStoreInstr(mStation->entry_instr.opcode))
            {
                if(currROBEntry->state == ISSUE)
                {
                    if (currStation->tag1 == UNDEFINED)
                    {
                        unsigned temp1Addr;
                        temp1Addr = currStation->value1 + currStation->entry_instr.immediate;
                        if ((temp1Addr) == (mStation->value2 + mStation->entry_instr.immediate))
                        {
                            mRetval = i;
                            break;
                        }
                    } else
                    {
                        mRetval = i;
                        break;
                    }
                }else if(currROBEntry->state == EXECUTE)
                {
                    if((currStation->address) == (mStation->value2 + mStation->entry_instr.immediate))
                    {
                        mRetval = i;
                        break;
                    }
                }else
                {
                    //Do Nothing
                }

            }
        }*/

    }
    return mRetval;

}

void update_instr_window(unsigned mPC, stage_t mStage)
{
    if(isValidPC(mPC))
    {
            for (int i = 0; i < currSim->pending_instructions.num_entries; i++)
            {
                if (currSim->pending_instructions.entries[i].pc == mPC)
                {
                    switch (mStage) {
                        case ISSUE:
                            currSim->pending_instructions.entries[i].issue = currClkCycle;
                            break;
                        case EXECUTE:
                            currSim->pending_instructions.entries[i].exe = currClkCycle;
                            break;
                        case WRITE_RESULT:
                            currSim->pending_instructions.entries[i].wr = currClkCycle;
                            break;
                        case COMMIT:
                            currSim->pending_instructions.entries[i].commit = currClkCycle;
                            break;
                        default:
                            //std::cout << "\n//TODO: error handling invalid stage";
                            break;
                    }
                    break;
                }
            }
    }
}

void store_bypassing_wb_handler(unsigned mPC)
{
    if(isValidPC(mPC))
    {
        instruction_t currInstr = currSim->instr_memory[(mPC - currSim->instr_base_address)/4];
        if((currInstr.opcode == SW) || (currInstr.opcode == SWS))
        {
            unsigned currStoreROBIndex = currSim->rob->get_entry_num(mPC);
            unsigned i = (currStoreROBIndex + 1)%currSim->rob->num_entries;
            if(i != currSim->rob->get_tail_index())
            {
                for(;i != currSim->rob->tailIndex; i = (i + 1)%currSim->rob->num_entries)
                {
                    rob_entry_t * currROBEntry = &currSim->rob->entries[i];
                    instruction_t currROBInstr = currROBEntry->entry_instr;
                    if(isLoadInstr(currROBInstr.opcode))
                    {
                        unsigned currLoadStationIndex;
                        currLoadStationIndex = currSim->reservation_stations->get_station_num(currROBEntry->pc);
                        if(currLoadStationIndex != UNDEFINED) {
                            res_station_entry_t *currStation = &currSim->reservation_stations->entries[currLoadStationIndex];
                            //this function is called after checking tags so values of stations should be available
                            if ((currStation->value1 + currROBInstr.immediate) ==
                                (currSim->rob->entries[currStoreROBIndex].destination)) {
                                if(currStation->value2 == UNDEFINED) {
                                    currStation->value2 = currSim->rob->entries[currStoreROBIndex].value;
                                    currStation->CDBWriteDataAvailClkCycle = currSim->pending_instructions.entries[currSim->rob->get_entry_num(mPC)].wr;
                                }
                            }
                        }
                    }
                }
            }else
            {
                //do nothing
            }

        }else
        {
            //std::cout << "\n//TODO: error handling invalid opcode for this function";
        }

    }else
    {
        //std::cout << "\n//TODO: error handling invalid PC";
    }
}

