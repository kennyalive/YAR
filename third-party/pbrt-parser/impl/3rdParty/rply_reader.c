/* ----------------------------------------------------------------------
 * RPly library, read/write PLY files
 * Diego Nehab, IMPA
 * http://www.impa.br/~diego/software/rply
 *
 * This library is distributed under the MIT License. See notice
 * at the end of this file.
 * ---------------------------------------------------------------------- */
#include <stdio.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>
#include <limits.h>
#include <float.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

#include "rply_reader.h"

/* ----------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */
#define WORDSIZE 256
#define LINESIZE 1024

static const char *const ply_type_list[] = {
    "int8", "uint8", "int16", "uint16",
    "int32", "uint32", "float32", "float64",
    "char", "uchar", "short", "ushort",
    "int", "uint", "float", "double",
    "list", NULL
};     /* order matches e_ply_type enum */

/* ----------------------------------------------------------------------
 * Property reading callback argument
 *
 * element: name of element being processed
 * property: name of property being processed
 * nelements: number of elements of this kind in file
 * instance_index: index current element of this kind being processed
 * length: number of values in current list (or 1 for scalars)
 * value_index: index of current value int this list (or 0 for scalars)
 * value: value of property
 * pdata/idata: user data defined with ply_set_cb
 *
 * Returns handle to PLY file if succesful, NULL otherwise.
 * ---------------------------------------------------------------------- */
typedef struct t_ply_argument_ {
    p_ply_element element;
    long instance_index;
    p_ply_property property;
    long length, value_index;
    double value;
    void *pdata;
    long idata;
} t_ply_argument;

/* ----------------------------------------------------------------------
 * Property information
 *
 * name: name of this property
 * type: type of this property (list or type of scalar value)
 * length_type, value_type: type of list property count and values
 * read_cb: function to be called when this property is called
 *
 * Returns 1 if should continue processing file, 0 if should abort.
 * ---------------------------------------------------------------------- */
typedef struct t_ply_property_ {
    char name[WORDSIZE];
    e_ply_type type, value_type, length_type;
    p_ply_read_cb read_cb;
    void *pdata;
    long idata;
} t_ply_property;

/* ----------------------------------------------------------------------
 * Element information
 *
 * name: name of this property
 * ninstances: number of elements of this type in file
 * property: property descriptions for this element
 * nproperty: number of properties in this element
 *
 * Returns 1 if should continue processing file, 0 if should abort.
 * ---------------------------------------------------------------------- */
typedef struct t_ply_element_ {
    char name[WORDSIZE];
    long ninstances;
    p_ply_property property;
    long nproperties;
} t_ply_element;


/* ----------------------------------------------------------------------
 * Ply file handle.
 *
  * element: elements description for this file
 * nelement: number of different elements in file
 * rn: skip extra char after end_header?
 * token_start: start of parsed token (line or word) in buffer
 * idriver, odriver: input driver used to get property fields from file
 * argument: storage space for callback arguments
 * welement, wproperty: element/property type being written
 * winstance_index: index of instance of current element being written
 * wvalue_index: index of list property value being written
 * wlength: number of values in list property being written
 * error_cb: error callback
 * pdata/idata: user data defined with ply_open/ply_create
 * ---------------------------------------------------------------------- */
typedef struct t_ply_ {
    p_ply_element element;
    long nelements;
    uint8_t* content;
    size_t content_size;
    size_t content_offset;
    int rn;
    size_t token_start;
    t_ply_argument argument;
    long welement, wproperty;
    long winstance_index, wvalue_index, wlength;
    p_ply_error_cb error_cb;
    void *pdata;
    long idata;
} t_ply;

/* ----------------------------------------------------------------------
 * I/O functions
 * ---------------------------------------------------------------------- */
static int ply_read_word(p_ply ply);
static int ply_check_word(p_ply ply);
static void ply_finish_word(p_ply ply, size_t size);
static int ply_read_line(p_ply ply);
static int ply_check_line(p_ply ply);

/* ----------------------------------------------------------------------
 * String functions
 * ---------------------------------------------------------------------- */
static int ply_find_string(const char *item, const char* const list[]);
static p_ply_element ply_find_element(p_ply ply, const char *name);
static p_ply_property ply_find_property(p_ply_element element,
        const char *name);

/* ----------------------------------------------------------------------
 * Header parsing
 * ---------------------------------------------------------------------- */
static int ply_read_header_magic(p_ply ply);
static int ply_read_header_format(p_ply ply);
static int ply_read_header_comment(p_ply ply);
static int ply_read_header_obj_info(p_ply ply);
static int ply_read_header_property(p_ply ply);
static int ply_read_header_element(p_ply ply);

/* ----------------------------------------------------------------------
 * Error handling
 * ---------------------------------------------------------------------- */
static void ply_error_cb(p_ply ply, const char *message);
static void ply_ferror(p_ply ply, const char *fmt, ...);

/* ----------------------------------------------------------------------
 * Memory allocation and initialization
 * ---------------------------------------------------------------------- */
static void ply_init(p_ply ply);
static void ply_element_init(p_ply_element element);
static void ply_property_init(p_ply_property property);
static p_ply ply_alloc(void);
static p_ply_element ply_grow_element(p_ply ply);
static p_ply_property ply_grow_property(p_ply ply, p_ply_element element);
static void *ply_grow_array(p_ply ply, void **pointer, long *nmemb, long size);

/* ----------------------------------------------------------------------
 * Auxiliary read functions
 * ---------------------------------------------------------------------- */
static int ply_read_element(p_ply ply, p_ply_element element,
        p_ply_argument argument);
static int ply_read_property(p_ply ply, p_ply_element element,
        p_ply_property property, p_ply_argument argument);
static int ply_read_list_property(p_ply ply, p_ply_element element,
        p_ply_property property, p_ply_argument argument);
static int ply_read_scalar_property(p_ply ply, p_ply_element element,
        p_ply_property property, p_ply_argument argument);

/* ----------------------------------------------------------------------
 * Buffer support functions
 * ---------------------------------------------------------------------- */
/* pointers to tokenized word and line in buffer */
#define BWORD(p) (p->content + p->token_start)
#define BLINE(p) (p->content + p->token_start)

/* pointer to start of untouched bytes in buffer */
#define BFIRST(p) (p->content + p->content_offset)

/* number of bytes untouched in buffer */
#define BSIZE(p) (p->content_size - p->content_offset)

/* consumes data from buffer */
#define BSKIP(p, s) (p->content_offset += s)

/* We don't care about end-of-line, generally, because we
 * separate words by any white-space character.
 * Unfortunately, in binary mode, right after 'end_header',
 * we have to know *exactly* how many characters to skip */
/* We use the end-of-line marker after the 'ply' magic
 * number to figure out what to do */
static int ply_read_header_magic(p_ply ply) {
    char* magic = (char*)ply->content + ply->content_offset;
    if (ply->content_offset + 3 >= ply->content_size) {
        ply->error_cb(ply, "Unable to read magic number");
        return 0;
    }
    /* check if it is ply */
    if (magic[0] != 'p' || magic[1] != 'l' || magic[2] != 'y'
            || !isspace(magic[3])) {
        ply->error_cb(ply, "Wrong magic number. Expected 'ply'");
        return 0;
    }
    /* figure out if we have to skip the extra character
     * after header when we reach the binary part of file */
    ply->rn = magic[3] == '\r' && magic[4] == '\n';
    BSKIP(ply, 3);
    return 1;
}

/* ----------------------------------------------------------------------
 * Exported functions
 * ---------------------------------------------------------------------- */
/* ----------------------------------------------------------------------
 * Read support functions
 * ---------------------------------------------------------------------- */
static p_ply ply_open_from_file(FILE* fp, p_ply_error_cb error_cb,
    long idata, void* pdata) {
    p_ply ply;
    if (error_cb == NULL) error_cb = ply_error_cb;
    assert(fp);

    if (fseek(fp, 0, SEEK_END) != 0) {
        return NULL;
    }
    size_t content_size = ftell(fp);
    if (fseek(fp, 0, SEEK_SET) != 0) {
        return NULL;
    }
    uint8_t* content = malloc(content_size);
    if (fread(content, 1, content_size, fp) != content_size) {
        free(content);
        return NULL;
    }

    ply = ply_alloc();
    if (!ply) {
        error_cb(NULL, "Out of memory");
        return NULL;
    }
    ply->idata = idata;
    ply->pdata = pdata;
    ply->error_cb = error_cb;
    ply->content = content;
    ply->content_size = content_size;
    ply->content_offset = 0;
    return ply;
}

p_ply ply_open(const char *name, p_ply_error_cb error_cb,
        long idata, void *pdata) {
    FILE *fp;
    p_ply ply;
    if (error_cb == NULL) error_cb = ply_error_cb;
    assert(name);
    fp = fopen(name, "rb");
    if (!fp) {
        error_cb(NULL, "Unable to open file");
        return NULL;
    }
    ply = ply_open_from_file(fp, error_cb, idata, pdata);
    fclose(fp);
    return ply;
}

int ply_read_header(p_ply ply) {
    assert(ply);
    if (!ply_read_header_magic(ply)) return 0;
    if (!ply_read_word(ply)) return 0;
    /* parse file format */
    if (!ply_read_header_format(ply)) {
        ply_ferror(ply, "Invalid file format");
        return 0;
    }
    /* parse elements, comments or obj_infos until the end of header */
    while (strcmp(BWORD(ply), "end_header")) {
        if (!ply_read_header_comment(ply) &&
                !ply_read_header_element(ply) &&
                !ply_read_header_obj_info(ply)) {
            ply_ferror(ply, "Unexpected token '%s'", BWORD(ply));
            return 0;
        }
    }
    /* skip extra character? */
    if (ply->rn) {
        if (BSIZE(ply) < 1) {
            ply_ferror(ply, "Unexpected end of file");
            return 0;
        }
        BSKIP(ply, 1);
    }
    return 1;
}

long ply_set_read_cb(p_ply ply, const char *element_name,
        const char* property_name, p_ply_read_cb read_cb,
        void *pdata, long idata) {
    p_ply_element element = NULL;
    p_ply_property property = NULL;
    assert(ply && element_name && property_name);
    element = ply_find_element(ply, element_name);
    if (!element) return 0;
    property = ply_find_property(element, property_name);
    if (!property) return 0;
    property->read_cb = read_cb;
    property->pdata = pdata;
    property->idata = idata;
    return (int) element->ninstances;
}

int ply_read(p_ply ply) {
    long i;
    p_ply_argument argument;
    assert(ply);
    argument = &ply->argument;
    /* for each element type */
    for (i = 0; i < ply->nelements; i++) {
        p_ply_element element = &ply->element[i];
        argument->element = element;
        if (!ply_read_element(ply, element, argument))
            return 0;
    }
    return 1;
}

static int ply_add_comment(p_ply ply, const char *comment) {
    char *new_comment = NULL;
    assert(ply && comment && strlen(comment) < LINESIZE);
    if (!comment || strlen(comment) >= LINESIZE) {
        ply_ferror(ply, "Invalid arguments");
        return 0;
    }
    // NOTE: do not support reading comments in this implementation
    return 1;
}

static int ply_add_obj_info(p_ply ply, const char *obj_info) {
    char *new_obj_info = NULL;
    assert(ply && obj_info && strlen(obj_info) < LINESIZE);
    if (!obj_info || strlen(obj_info) >= LINESIZE) {
        ply_ferror(ply, "Invalid arguments");
        return 0;
    }
    // NOTE: do not support reading obj_info in this implementation
    return 1;
}

int ply_close(p_ply ply) {
    long i;
    assert(ply);
    assert(ply->element || ply->nelements == 0);
    assert(!ply->element || ply->nelements > 0);
    /* free all memory used by handle */
    if (ply->element) {
        for (i = 0; i < ply->nelements; i++) {
            p_ply_element element = &ply->element[i];
            if (element->property) free(element->property);
        }
        free(ply->element);
    }
    free(ply->content);
    free(ply);
    return 1;
}

/* ----------------------------------------------------------------------
 * Query support functions
 * ---------------------------------------------------------------------- */
p_ply_element ply_get_next_element(p_ply ply,
        p_ply_element last) {
    assert(ply);
    if (!last) return ply->element;
    last++;
    if (last < ply->element + ply->nelements) return last;
    else return NULL;
}

int ply_get_element_info(p_ply_element element, const char** name,
        long *ninstances) {
    assert(element);
    if (name) *name = element->name;
    if (ninstances) *ninstances = (long) element->ninstances;
    return 1;
}

p_ply_property ply_get_next_property(p_ply_element element,
        p_ply_property last) {
    assert(element);
    if (!last) return element->property;
    last++;
    if (last < element->property + element->nproperties) return last;
    else return NULL;
}

int ply_get_property_info(p_ply_property property, const char** name,
        e_ply_type *type, e_ply_type *length_type, e_ply_type *value_type) {
    assert(property);
    if (name) *name = property->name;
    if (type) *type = property->type;
    if (length_type) *length_type = property->length_type;
    if (value_type) *value_type = property->value_type;
    return 1;

}

/* ----------------------------------------------------------------------
 * Callback argument support functions
 * ---------------------------------------------------------------------- */
int ply_get_argument_element(p_ply_argument argument,
        p_ply_element *element, long *instance_index) {
    assert(argument);
    if (!argument) return 0;
    if (element) *element = argument->element;
    if (instance_index) *instance_index = argument->instance_index;
    return 1;
}

int ply_get_argument_property(p_ply_argument argument,
        p_ply_property *property, long *length, long *value_index) {
    assert(argument);
    if (!argument) return 0;
    if (property) *property = argument->property;
    if (length) *length = argument->length;
    if (value_index) *value_index = argument->value_index;
    return 1;
}

int ply_get_argument_user_data(p_ply_argument argument, void **pdata,
        long *idata) {
    assert(argument);
    if (!argument) return 0;
    if (pdata) *pdata = argument->pdata;
    if (idata) *idata = argument->idata;
    return 1;
}

double ply_get_argument_value(p_ply_argument argument) {
    assert(argument);
    if (!argument) return 0.0;
    return argument->value;
}

int ply_get_ply_user_data(p_ply ply, void **pdata, long *idata) {
    assert(ply);
    if (!ply) return 0;
    if (pdata) *pdata = ply->pdata;
    if (idata) *idata = ply->idata;
    return 1;
}

/* ----------------------------------------------------------------------
 * Internal functions
 * ---------------------------------------------------------------------- */
static int binary_int8(p_ply ply, double* value) {
    if (BSIZE(ply) < 1) {
        return 0;
    }
    int8_t int8;
    memcpy(&int8, BFIRST(ply), 1);
    BSKIP(ply, 1);
    *value = int8;
    return 1;
}

static int binary_uint8(p_ply ply, double* value) {
    if (BSIZE(ply) < 1) {
        return 0;
    }
    uint8_t uint8;
    memcpy(&uint8, BFIRST(ply), 1);
    BSKIP(ply, 1);
    *value = uint8;
    return 1;
}

static int binary_int16(p_ply ply, double* value) {
    if (BSIZE(ply) < 2) {
        return 0;
    }
    int16_t int16;
    memcpy(&int16, BFIRST(ply), 2);
    BSKIP(ply, 2);
    *value = int16;
    return 1;
}

static int binary_uint16(p_ply ply, double* value) {
    if (BSIZE(ply) < 2) {
        return 0;
    }
    uint16_t uint16;
    memcpy(&uint16, BFIRST(ply), 2);
    BSKIP(ply, 2);
    *value = uint16;
    return 1;
}

static int binary_int32(p_ply ply, double* value) {
    if (BSIZE(ply) < 4) {
        return 0;
    }
    int32_t int32;
    memcpy(&int32, BFIRST(ply), 4);
    BSKIP(ply, 4);
    *value = int32;
    return 1;
}

static int binary_uint32(p_ply ply, double* value) {
    if (BSIZE(ply) < 4) {
        return 0;
    }
    uint32_t uint32;
    memcpy(&uint32, BFIRST(ply), 4);
    BSKIP(ply, 4);
    *value = uint32;
    return 1;
}

static int binary_float32(p_ply ply, double* value) {
    if (BSIZE(ply) < 4) {
        return 0;
    }
    float float32;
    memcpy(&float32, BFIRST(ply), 4);
    BSKIP(ply, 4);
    *value = float32;
    return 1;
}

static int binary_float64(p_ply ply, double* value) {
    if (BSIZE(ply) < 8) {
        return 0;
    }
    memcpy(value, BFIRST(ply), 8);
    BSKIP(ply, 8);
    return 1;
}

typedef int (*p_ply_type_handler)(p_ply ply, double* value);

static p_ply_type_handler ply_type_handlers[16] = {
    binary_int8, binary_uint8, binary_int16, binary_uint16,
    binary_int32, binary_uint32, binary_float32, binary_float64,
    binary_int8, binary_uint8, binary_int16, binary_uint16,
    binary_int32, binary_uint32, binary_float32, binary_float64
};

static int ply_read_list_property(p_ply ply, p_ply_element element,
        p_ply_property property, p_ply_argument argument) {
    int l;
    p_ply_read_cb read_cb = property->read_cb;
    /* get list length */
    p_ply_type_handler handler = ply_type_handlers[property->length_type];
    double length;
    if (!handler(ply, &length)) {
        ply_ferror(ply, "Error reading '%s' of '%s' number %d",
                property->name, element->name, argument->instance_index);
        return 0;
    }
    /* invoke callback to pass length in value field */
    argument->length = (long) length;
    argument->value_index = -1;
    argument->value = length;
    if (read_cb && !read_cb(argument)) {
        ply_ferror(ply, "Aborted by user");
        return 0;
    }
    /* read list values */
    handler = ply_type_handlers[property->value_type];
    /* for each value in list */
    for (l = 0; l < (long) length; l++) {
        /* read value from file */
        argument->value_index = l;
        if (!handler(ply, &argument->value)) {
            ply_ferror(ply, "Error reading value number %d of '%s' of "
                    "'%s' number %d", l+1, property->name,
                    element->name, argument->instance_index);
            return 0;
        }
        /* invoke callback to pass value */
        if (read_cb && !read_cb(argument)) {
            ply_ferror(ply, "Aborted by user");
            return 0;
        }
    }
    return 1;
}

static int ply_read_scalar_property(p_ply ply, p_ply_element element,
        p_ply_property property, p_ply_argument argument) {
    p_ply_read_cb read_cb = property->read_cb;
    p_ply_type_handler handler = ply_type_handlers[property->type];
    argument->length = 1;
    argument->value_index = 0;
    if (!handler(ply, &argument->value)) {
        ply_ferror(ply, "Error reading '%s' of '%s' number %d",
                property->name, element->name, argument->instance_index);
        return 0;
    }
    if (read_cb && !read_cb(argument)) {
        ply_ferror(ply, "Aborted by user");
        return 0;
    }
    return 1;
}

static int ply_read_property(p_ply ply, p_ply_element element,
        p_ply_property property, p_ply_argument argument) {
    if (property->type == PLY_LIST)
        return ply_read_list_property(ply, element, property, argument);
    else
        return ply_read_scalar_property(ply, element, property, argument);
}

static int ply_read_element(p_ply ply, p_ply_element element,
        p_ply_argument argument) {
    long j, k;
    /* for each element of this type */
    for (j = 0; j < element->ninstances; j++) {
        argument->instance_index = j;
        /* for each property */
        for (k = 0; k < element->nproperties; k++) {
            p_ply_property property = &element->property[k];
            argument->property = property;
            argument->pdata = property->pdata;
            argument->idata = property->idata;
            if (!ply_read_property(ply, element, property, argument))
                return 0;
        }
    }
    return 1;
}

static int ply_find_string(const char *item, const char* const list[]) {
    int i;
    assert(item && list);
    for (i = 0; list[i]; i++)
        if (!strcmp(list[i], item)) return i;
    return -1;
}

static p_ply_element ply_find_element(p_ply ply, const char *name) {
    p_ply_element element;
    int i, nelements;
    assert(ply && name);
    element = ply->element;
    nelements = ply->nelements;
    assert(element || nelements == 0);
    assert(!element || nelements > 0);
    for (i = 0; i < nelements; i++)
        if (!strcmp(element[i].name, name)) return &element[i];
    return NULL;
}

static p_ply_property ply_find_property(p_ply_element element,
        const char *name) {
    p_ply_property property;
    int i, nproperties;
    assert(element && name);
    property = element->property;
    nproperties = element->nproperties;
    assert(property || nproperties == 0);
    assert(!property || nproperties > 0);
    for (i = 0; i < nproperties; i++)
        if (!strcmp(property[i].name, name)) return &property[i];
    return NULL;
}

static int ply_check_word(p_ply ply) {
    size_t size = strlen(BWORD(ply));
    if (size >= WORDSIZE) {
        ply_ferror(ply, "Word too long");
        return 0;
    } else if (size == 0) {
        ply_ferror(ply, "Unexpected end of file");
        return 0;
    }
    return 1;
}

static int ply_read_word(p_ply ply) {
    size_t t = 0;
    assert(ply);
    /* skip leading blanks */
    while (1) {
        t = strspn(BFIRST(ply), " \n\r\t");
        /* check if all buffer was made of blanks */
        if (t >= BSIZE(ply)) {
            ply_ferror(ply, "Unexpected end of file");
            return 0;
        } else break;
    }
    BSKIP(ply, t);
    /* look for a space after the current word */
    t = strcspn(BFIRST(ply), " \n\r\t");
    ply_finish_word(ply, t);
    return ply_check_word(ply);
}

static void ply_finish_word(p_ply ply, size_t size) {
    ply->token_start = ply->content_offset;
    BSKIP(ply, size);
    *BFIRST(ply) = '\0';
    BSKIP(ply, 1);
}

static int ply_check_line(p_ply ply) {
    if (strlen(BLINE(ply)) >= LINESIZE) {
        ply_ferror(ply, "Line too long");
        return 0;
    }
    return 1;
}

static int ply_read_line(p_ply ply) {
    const char *end = NULL;
    assert(ply);
    /* look for a end of line */
    end = strchr(BFIRST(ply), '\n');
    /* if we didn't reach the end of the buffer, we are done */
    if (end) {
        ply->token_start = ply->content_offset;
        BSKIP(ply, end - BFIRST(ply));
        *BFIRST(ply) = '\0';
        BSKIP(ply, 1);
        return ply_check_line(ply);
    } else {
        ply_ferror(ply, "Unexpected end of file");
        return 0;
    }
    /* keep looking from where we left */
    end = strchr(end, '\n');
    /* check if the token is too large for our buffer */
    if (!end) {
        ply_ferror(ply, "Token too large");
        return 0;
    }
    /* we are done */
    ply->token_start = ply->content_offset;
    BSKIP(ply, end - BFIRST(ply));
    *BFIRST(ply) = '\0';
    BSKIP(ply, 1);
    return ply_check_line(ply);
}

static void ply_init(p_ply ply) {
    ply->element = NULL;
    ply->nelements = 0;
    ply->content = NULL;
    ply->content_size = 0;
    ply->content_offset = 0;
    ply->token_start = 0;
    ply->welement = 0;
    ply->wproperty = 0;
    ply->winstance_index = 0;
    ply->wlength = 0;
    ply->wvalue_index = 0;
}

static void ply_element_init(p_ply_element element) {
    element->name[0] = '\0';
    element->ninstances = 0;
    element->property = NULL;
    element->nproperties = 0;
}

static void ply_property_init(p_ply_property property) {
    property->name[0] = '\0';
    property->type = -1;
    property->length_type = -1;
    property->value_type = -1;
    property->read_cb = (p_ply_read_cb) NULL;
    property->pdata = NULL;
    property->idata = 0;
}

static p_ply ply_alloc(void) {
    p_ply ply = (p_ply) calloc(1, sizeof(t_ply));
    if (!ply) return NULL;
    ply_init(ply);
    return ply;
}

static void *ply_grow_array(p_ply ply, void **pointer,
        long *nmemb, long size) {
    void *temp = *pointer;
    long count = *nmemb + 1;
    if (!temp) temp = malloc(count*size);
    else temp = realloc(temp, count*size);
    if (!temp) {
        ply_ferror(ply, "Out of memory");
        return NULL;
    }
    *pointer = temp;
    *nmemb = count;
    return (char *) temp + (count-1) * size;
}

static p_ply_element ply_grow_element(p_ply ply) {
    p_ply_element element = NULL;
    assert(ply);
    assert(ply->element || ply->nelements == 0);
    assert(!ply->element || ply->nelements > 0);
    element = (p_ply_element) ply_grow_array(ply, (void **) &ply->element,
            &ply->nelements, sizeof(t_ply_element));
    if (!element) return NULL;
    ply_element_init(element);
    return element;
}

static p_ply_property ply_grow_property(p_ply ply, p_ply_element element) {
    p_ply_property property = NULL;
    assert(ply);
    assert(element);
    assert(element->property || element->nproperties == 0);
    assert(!element->property || element->nproperties > 0);
    property = (p_ply_property) ply_grow_array(ply,
            (void **) &element->property,
            &element->nproperties, sizeof(t_ply_property));
    if (!property) return NULL;
    ply_property_init(property);
    return property;
}

static int ply_read_header_format(p_ply ply) {
    assert(ply);
    if (strcmp(BWORD(ply), "format")) return 0;
    if (!ply_read_word(ply)) return 0;
    if (strcmp(BWORD(ply), "binary_little_endian") != 0) {
        // NOTE: this implementation supports only binary little endian ply files
        return 0;
    }
    if (!ply_read_word(ply)) return 0;
    if (strcmp(BWORD(ply), "1.0")) return 0;
    if (!ply_read_word(ply)) return 0;
    return 1;
}

static int ply_read_header_comment(p_ply ply) {
    assert(ply);
    if (strcmp(BWORD(ply), "comment")) return 0;
    if (!ply_read_line(ply)) return 0;
    if (!ply_add_comment(ply, BLINE(ply))) return 0;
    if (!ply_read_word(ply)) return 0;
    return 1;
}

static int ply_read_header_obj_info(p_ply ply) {
    assert(ply);
    if (strcmp(BWORD(ply), "obj_info")) return 0;
    if (!ply_read_line(ply)) return 0;
    if (!ply_add_obj_info(ply, BLINE(ply))) return 0;
    if (!ply_read_word(ply)) return 0;
    return 1;
}

static int ply_read_header_property(p_ply ply) {
    p_ply_element element = NULL;
    p_ply_property property = NULL;
    /* make sure it is a property */
    if (strcmp(BWORD(ply), "property")) return 0;
    element = &ply->element[ply->nelements-1];
    property = ply_grow_property(ply, element);
    if (!property) return 0;
    /* get property type */
    if (!ply_read_word(ply)) return 0;
    property->type = ply_find_string(BWORD(ply), ply_type_list);
    if (property->type == (e_ply_type) (-1)) return 0;
    if (property->type == PLY_LIST) {
        /* if it's a list, we need the base types */
        if (!ply_read_word(ply)) return 0;
        property->length_type = ply_find_string(BWORD(ply), ply_type_list);
        if (property->length_type == (e_ply_type) (-1)) return 0;
        if (!ply_read_word(ply)) return 0;
        property->value_type = ply_find_string(BWORD(ply), ply_type_list);
        if (property->value_type == (e_ply_type) (-1)) return 0;
    }
    /* get property name */
    if (!ply_read_word(ply)) return 0;
    strcpy(property->name, BWORD(ply));
    if (!ply_read_word(ply)) return 0;
    return 1;
}

static int ply_read_header_element(p_ply ply) {
    p_ply_element element = NULL;
    long dummy;
    assert(ply);
    if (strcmp(BWORD(ply), "element")) return 0;
    /* allocate room for new element */
    element = ply_grow_element(ply);
    if (!element) return 0;
    /* get element name */
    if (!ply_read_word(ply)) return 0;
    strcpy(element->name, BWORD(ply));
    /* get number of elements of this type */
    if (!ply_read_word(ply)) return 0;
    if (sscanf(BWORD(ply), "%ld", &dummy) != 1) {
        ply_ferror(ply, "Expected number got '%s'", BWORD(ply));
        return 0;
    }
    element->ninstances = dummy;
    /* get all properties for this element */
    if (!ply_read_word(ply)) return 0;
    while (ply_read_header_property(ply) ||
        ply_read_header_comment(ply) || ply_read_header_obj_info(ply))
        /* do nothing */;
    return 1;
}

static void ply_error_cb(p_ply ply, const char *message) {
    (void) ply;
    fprintf(stderr, "RPly: %s\n", message);
}

static void ply_ferror(p_ply ply, const char *fmt, ...) {
    char buffer[1024];
    va_list ap;
    va_start(ap, fmt);
    vsprintf(buffer, fmt, ap);
    va_end(ap);
    ply->error_cb(ply, buffer);
}

/* ----------------------------------------------------------------------
 * Copyright (C) 2003-2015 Diego Nehab.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * ---------------------------------------------------------------------- */
