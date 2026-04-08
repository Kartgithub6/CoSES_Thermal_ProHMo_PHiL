#include <windows.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define NI_OK    0
#define NI_ERROR 1

#ifdef __cplusplus
#define DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllexport)
#endif

typedef struct { uint32_t major, minor, fix, build; } NI_Version;
DLL_EXPORT NI_Version NIVS_APIversion = {2018, 0, 0, 81};

typedef void*        fmi2Comp;
typedef unsigned int fmi2VR;
typedef void (*fmi2Logger)(void*, const char*, int, const char*, const char*, ...);
typedef void* (*fmi2Alloc)(size_t, size_t);
typedef void  (*fmi2Free)(void*);
typedef struct {
    fmi2Logger logger; fmi2Alloc alloc; fmi2Free free; void* sf; void* ce;
} fmi2CB;
typedef fmi2Comp (*pFn_Instantiate)(const char*, int, const char*, const char*, const fmi2CB*, int, int);
typedef int      (*pFn_Setup)      (fmi2Comp, int, double, double, int, double);
typedef int      (*pFn_EnterInit)  (fmi2Comp);
typedef int      (*pFn_ExitInit)   (fmi2Comp);
typedef int      (*pFn_DoStep)     (fmi2Comp, double, double, int);
typedef int      (*pFn_SetReal)    (fmi2Comp, const fmi2VR*, size_t, const double*);
typedef int      (*pFn_GetReal)    (fmi2Comp, const fmi2VR*, size_t, double*);
typedef int      (*pFn_Terminate)  (fmi2Comp);
typedef void     (*pFn_Free)       (fmi2Comp);

#define FMU_DLL  "tzb_fmu.dll"
#define FMU_GUID "{0c4fc5dd-8578-4e04-84d8-dd2e48eec986}"
#define STEP     0.01
#define N_IN     9
#define N_OUT    11

/* VRs from ~FMUOutput modelDescription.xml */
static fmi2VR vr_in[9]  = {352321536,352321537,352321538,352321539,352321540,
                            352321541,352321542,352321543,352321544};
static fmi2VR vr_out[11] = {335544320,335544321,335544322,335544323,
                             335544324,335544325,335544326,335544327,
                             637537454,637537317,637537180};

static const char* IN_NAMES[9] = {
    "STM_HCVLaM_degC","SFW_HCRLbM_l_per_min","T_ambient_degC",
    "nPersons_living_in","nPersons_cellar_in","nPersons_roof_in",
    "P_appliances_living_W_in","P_appliances_cellar_W_in","P_appliances_roof_W_in"
};
static const char* OUT_NAMES[11] = {
    "T_roomIs_degC","T_cellarIs_degC","T_roofIs_degC","STM_HCRL_Set_degC",
    "SFW_HCRLbM_Set_l_per_min","valve_cellar_opening","valve_living_opening","valve_roof_opening",
    "Q_flow_roof_W","Q_flow_living_W","Q_flow_cellar_W"
};

static HMODULE         g_dll  = NULL;
static fmi2Comp        g_comp = NULL;
static double          g_time = 0.0;
static pFn_Instantiate g_Inst = NULL;
static pFn_Setup       g_Setup = NULL;
static pFn_EnterInit   g_EI = NULL;
static pFn_ExitInit    g_XI = NULL;
static pFn_DoStep      g_DS = NULL;
static pFn_SetReal     g_SR = NULL;
static pFn_GetReal     g_GR = NULL;
static pFn_Terminate   g_Term = NULL;
static pFn_Free        g_FI = NULL;
static char            g_uri[MAX_PATH+10] = {0};
static fmi2CB          g_cb = {NULL,NULL,NULL,NULL,NULL};

static void dummyLog(void* e, const char* i, int s, const char* c, const char* m, ...) {
    (void)e;(void)i;(void)s;(void)c;(void)m;
}

static void dbg(const char* msg) {
    FILE* f = fopen("C:\\temp\\tzb.txt", "a");
    if (f) { fprintf(f, "%s\n", msg); fclose(f); }
}

static int32_t initFMU(void) {
    char dir[MAX_PATH]={0}, dll[MAX_PATH]={0}, uri[MAX_PATH+10]={0}, tmp[MAX_PATH]={0};
    fmi2CB cb = {dummyLog, calloc, free, NULL, NULL};
    HMODULE hSelf = NULL;
    DWORD err;

    CreateDirectoryA("C:\\temp", NULL);
    dbg("initFMU START");

    GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        (LPCSTR)&initFMU, &hSelf);
    GetModuleFileNameA(hSelf, dir, MAX_PATH);
    { char* p = strrchr(dir, 92); if (p) *(p+1) = 0; }
    dbg(dir);

    SetDllDirectoryA(dir);
    strncpy(dll, dir, MAX_PATH-1);
    strncat(dll, FMU_DLL, MAX_PATH - strlen(dll) - 1);
    dbg(dll);

    g_dll = LoadLibraryA(dll);
    if (!g_dll) g_dll = GetModuleHandleA(dll);
    if (!g_dll) {
        err = GetLastError();
        sprintf(tmp, "LoadLib FAIL %lu", err);
        dbg(tmp);
        return NI_ERROR;
    }
    dbg("DLL OK");

    g_Inst  = (pFn_Instantiate) GetProcAddress(g_dll, "fmi2Instantiate");
    g_Setup = (pFn_Setup)       GetProcAddress(g_dll, "fmi2SetupExperiment");
    g_EI    = (pFn_EnterInit)   GetProcAddress(g_dll, "fmi2EnterInitializationMode");
    g_XI    = (pFn_ExitInit)    GetProcAddress(g_dll, "fmi2ExitInitializationMode");
    g_DS    = (pFn_DoStep)      GetProcAddress(g_dll, "fmi2DoStep");
    g_SR    = (pFn_SetReal)     GetProcAddress(g_dll, "fmi2SetReal");
    g_GR    = (pFn_GetReal)     GetProcAddress(g_dll, "fmi2GetReal");
    g_Term  = (pFn_Terminate)   GetProcAddress(g_dll, "fmi2Terminate");
    g_FI    = (pFn_Free)        GetProcAddress(g_dll, "fmi2FreeInstance");

    sprintf(tmp, "Inst=%p DS=%p SR=%p GR=%p", (void*)g_Inst, (void*)g_DS, (void*)g_SR, (void*)g_GR);
    dbg(tmp);

    if (!g_Inst || !g_DS || !g_SR || !g_GR) { dbg("MISSING FUNCS"); return NI_ERROR; }

    strcpy(uri, "file:///D:/coses_main/Veristand/Models/HostPC/ThreeZoneBuilding_PHiL_optimized_withWeather");
    strcpy(g_uri, uri);
    cb.logger=dummyLog; cb.alloc=calloc; cb.free=free; cb.sf=NULL; cb.ce=NULL;
    g_cb = cb;
    dbg(uri);

    g_comp = g_Inst("ThreeZoneBuilding", 1, FMU_GUID, uri, &cb, 0, 1);
    sprintf(tmp, "comp=%p", (void*)g_comp);
    dbg(tmp);
    if (!g_comp) { dbg("Instantiate NULL"); return NI_ERROR; }

    if (g_Setup) g_Setup(g_comp, 0, 0.0, 1e10, 0, 0.0);
    if (g_EI)    g_EI(g_comp);
    if (g_XI)    g_XI(g_comp);
    g_time = 0.0;
    dbg("initFMU SUCCESS");
    return NI_OK;
}

static const char BUILD_INFO[] =
    "ThreeZoneBuilding_PHiL\n"
    "NI VeriStand version: 2018.0.0.81\n"
    "Bit-per-pointer: 32\n";

DLL_EXPORT int32_t NIRT_GetBuildInfo(char* detail, int32_t* len) {
    int32_t blen = (int32_t)strlen(BUILD_INFO);
    dbg("GetBuildInfo called");
    if (!len) return NI_OK;
    if (*len == -1 || !detail) { *len = blen + 1; return NI_OK; }
    strncpy(detail, BUILD_INFO, *len);
    *len = blen;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetModelSpec(char* name, int32_t* namelen, double* baseTimeStep,
    int32_t* outNumInports, int32_t* outNumOutports, int32_t* numtasks) {
    const char* mn = "ThreeZoneBuilding_PHiL";
    int32_t ml = (int32_t)strlen(mn);
    dbg("GetModelSpec called");
    if (namelen) {
        if (*namelen == -1 || !name) { *namelen = ml + 1; }
        else { strncpy(name, mn, *namelen); *namelen = ml; }
    }
    if (baseTimeStep)   *baseTimeStep   = STEP;
    if (outNumInports)  *outNumInports  = N_IN;
    if (outNumOutports) *outNumOutports = N_OUT;
    if (numtasks)       *numtasks       = 1;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_InitializeModel(double finaltime, double* outTimeStep,
    int32_t* num_in, int32_t* num_out, int32_t* num_tasks) {
    int32_t r;
    dbg("InitializeModel called");
    r = initFMU();
    if (outTimeStep) *outTimeStep = STEP;
    if (num_in)      *num_in      = N_IN;
    if (num_out)     *num_out     = N_OUT;
    if (num_tasks)   *num_tasks   = 1;
    return r;
}

DLL_EXPORT int32_t NIRT_ModelStart(void) {
    dbg("ModelStart called");
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_Schedule(double* inData, double* outData, double* outTime, int32_t* dispatchtasks) {
    char tmp[256];
    dbg("Schedule called");
    if (!g_comp) { dbg("Schedule: g_comp NULL"); return NI_ERROR; }
    if (!g_SR)   { dbg("Schedule: g_SR NULL");   return NI_ERROR; }
    if (!g_DS)   { dbg("Schedule: g_DS NULL");   return NI_ERROR; }
    if (!g_GR)   { dbg("Schedule: g_GR NULL");   return NI_ERROR; }
    if (!inData) { dbg("Schedule: inData NULL");  return NI_ERROR; }
    if (!outData){ dbg("Schedule: outData NULL"); return NI_ERROR; }

    sprintf(tmp,"Schedule: t=%.4f",g_time); dbg(tmp);

    {
        int i;
        double safe[N_IN];
        /* Default values in degC - same units as VeriStand sends */
        double def[N_IN] = {45.0, 12.0, 2.0, 2.0, 0.0, 1.0, 300.0, 50.0, 100.0};
        for(i=0; i<N_IN; i++){
            double v = inData[i];
            if(v!=v || v>1e15 || v<-1e15) v=def[i];
            safe[i]=v;
        }
        dbg("Schedule: calling SetReal");
        g_SR(g_comp, vr_in, N_IN, safe);
        dbg("Schedule: SetReal OK");
		sprintf(tmp,"in: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",safe[0],safe[1],safe[2],safe[3],safe[4],safe[5],safe[6],safe[7],safe[8]); dbg(tmp);
        dbg("Schedule: calling DoStep");
        __try {
			if (g_time > 0.0) {
				g_DS(g_comp, g_time, STEP, 1);
				dbg("Schedule: DoStep OK");
			} else {
				dbg("Schedule: DoStep SKIPPED at t=0");}
		} __except(EXCEPTION_EXECUTE_HANDLER) {
			dbg("Schedule: DoStep EXCEPTION - reinitializing DLL");
			g_comp = NULL;
			g_comp = g_Inst("ThreeZoneBuilding",1,FMU_GUID,g_uri,&g_cb,0,0);
			if(g_comp){
				if(g_Setup) g_Setup(g_comp,0,0.0,1e10,0,0.0);
				if(g_EI) g_EI(g_comp);
				if(g_XI) g_XI(g_comp);
				g_time=0.0;
				dbg("Schedule: FMU reinitialized");
			}
			if (outTime) *outTime = g_time;
			return NI_OK;
		}

        g_time += STEP;
        dbg("Schedule: calling GetReal");
        g_GR(g_comp, vr_out, N_OUT, outData);
        sprintf(tmp, "Schedule: GetReal vals: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
            outData[0],outData[1],outData[2],outData[3],
            outData[4],outData[5],outData[6],outData[7],
            outData[8],outData[9],outData[10]);
        dbg(tmp);
        dbg("Schedule: GetReal OK");
    }
    if (outTime) *outTime = g_time;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_PostOutputs(double* outData) { return NI_OK; }
DLL_EXPORT int32_t NIRT_ModelUpdate(void) { return NI_OK; }

DLL_EXPORT int32_t NIRT_FinalizeModel(void) {
    dbg("FinalizeModel called");
    g_comp = NULL;
    if (g_dll) { FreeLibrary(g_dll); g_dll = NULL; }
    g_Inst=NULL; g_Setup=NULL; g_EI=NULL; g_XI=NULL;
    g_DS=NULL; g_SR=NULL; g_GR=NULL; g_Term=NULL; g_FI=NULL;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetExtIOSpec(int32_t index, int32_t* idx, char* name,
    int32_t* tid, int32_t* type, int32_t* dims, int32_t* numdims) {
    int32_t tot = N_IN + N_OUT;
    dbg("GetExtIOSpec called");
    if (index == -1) return tot;
    if (index < 0 || index >= tot) return NI_ERROR;
    if (idx) *idx = index;
    if (tid) *tid = 0;
    if (index < N_IN) {
        if (type) *type = 0;
        if (name) strncpy(name, IN_NAMES[index], 256);
    } else {
        if (type) *type = 1;
        if (name) strncpy(name, OUT_NAMES[index - N_IN], 256);
    }
    if (numdims) {
        if (*numdims == -1) { *numdims = 2; return NI_OK; }
        if (dims && *numdims >= 2) { dims[0] = 1; dims[1] = 1; }
        *numdims = 2;
    }
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetModelFrameworkVersion(uint32_t* major, uint32_t* minor, uint32_t* fix, uint32_t* build) {
    if (major) *major = 2018;
    if (minor) *minor = 0;
    if (fix)   *fix   = 0;
    if (build) *build = 81;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_ModelError(char* errmsg, int32_t* msglen) {
    if (msglen) *msglen = 0;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetErrorMessageLength(void) { return 0; }

DLL_EXPORT int32_t NIRT_ProbeSignals(int32_t* sigindices, int32_t numsigs, double* value, int32_t* num) {
    if (num) *num = 0;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetParameterIndices(int32_t* indices, int32_t* len) {
    if (len) *len = 0;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetParameterSpec(int32_t* paramidx, char* ID, int32_t* ID_len,
    char* paramname, int32_t* pnlen, int32_t* datatype, int32_t* dims, int32_t* numdim) {
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetSignalSpec(int32_t* sigidx, char* ID, int32_t* ID_len,
    char* blkname, int32_t* bnlen, int32_t* portnum, char* signame, int32_t* snlen,
    int32_t* datatype, int32_t* dims, int32_t* numdim) {
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetParameter(int32_t index, int32_t subindex, double* val) { return NI_OK; }
DLL_EXPORT int32_t NIRT_SetParameter(int32_t index, int32_t subindex, double val)  { return NI_OK; }
DLL_EXPORT int32_t NIRT_SetScalarParameterInline(uint32_t index, uint32_t subindex, double paramvalue) { return NI_OK; }
DLL_EXPORT int32_t NIRT_SetVectorParameter(uint32_t index, const double* paramvalues, uint32_t paramlength) { return NI_OK; }
DLL_EXPORT int32_t NIRT_GetVectorParameter(uint32_t index, double* paramValues, uint32_t paramLength) { return NI_OK; }
DLL_EXPORT int32_t NIRT_TaskTakeOneStep(int32_t taskid) { return NI_OK; }

DLL_EXPORT int32_t NIRT_TaskRunTimeInfo(int32_t halt, int32_t* overruns, int32_t* numtasks) {
    if (overruns) *overruns = 0;
    if (numtasks) *numtasks = 1;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetTaskSpec(int32_t index, int32_t* tid, double* tstep, double* offset) {
    dbg("GetTaskSpec called");
    if (index == -1) return 1;
    if (tid)    *tid    = 0;
    if (tstep)  *tstep  = STEP;
    if (offset) *offset = 0.0;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_GetSimState(int32_t* numContStates, char* contStatesNames, double* contStates,
    int32_t* numDiscStates, char* discStatesNames, double* discStates,
    int32_t* numClockTicks, char* clockTicksNames, int32_t* clockTicks) {
    if (numContStates) *numContStates = 0;
    if (numDiscStates) *numDiscStates = 0;
    if (numClockTicks) *numClockTicks = 0;
    return NI_OK;
}

DLL_EXPORT int32_t NIRT_SetSimState(double* contStates, double* discStates, int32_t* clockTicks) {
    return NI_OK;
}