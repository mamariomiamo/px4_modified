from xml.sax.saxutils import escape
import codecs

class MarkdownTablesOutput():
    def __init__(self, groups):
        result = ("# Parameter Reference\n"
                  "> **Note** **This list is auto-generated from the source code** (using `make parameters_metadata`) and contains the most recent parameter documentation.\n"
                  "\n")
        for group in groups:
            result += '## %s\n\n' % group.GetName()
            
            #Check if scope (module where parameter is defined) is the same for all parameters in the group. 
            # If so then display just once about the table. 
            scope_set = set()
            for param in group.GetParams():
                scope_set.add(param.GetFieldValue("scope"))
            if len(scope_set)==1:
                result+='\nThe module where these parameters are defined is: *%s*.\n\n' %  list(scope_set)[0]
            

            for param in group.GetParams():
                code = param.GetName()
                name = param.GetFieldValue("short_desc") or ''
                long_desc = param.GetFieldValue("long_desc") or ''
                min_val = param.GetFieldValue("min") or ''
                max_val = param.GetFieldValue("max") or ''
                increment = param.GetFieldValue("increment") or ''
                def_val = param.GetDefault() or ''
                unit = param.GetFieldValue("unit") or 'NA'
                type = param.GetType()
                reboot_required = param.GetFieldValue("reboot_required") or ''
                #board = param.GetFieldValue("board") or '' ## Disabled as no board values are defined in any parameters!
                #decimal = param.GetFieldValue("decimal") or '' #Disabled as is intended for GCS not people
                #field_codes = param.GetFieldCodes() ## Disabled as not needed for display. 
                #boolean = param.GetFieldValue("boolean") # or '' # Disabled - does not appear useful.


                # Format values for display.
                # Display min/max/increment value based on what values are defined.
                max_min_combined = ''
                if min_val or max_val:
                    if not min_val:
                        min_val='?'
                    if not max_val:
                        max_val='?'
                    max_min_combined+='%s > %s ' % (min_val, max_val)
                else: 
                    max_min_combined+= 'NA'
                if increment:
                    max_min_combined+='(%s)' % increment

                if long_desc is not '':
                    long_desc = '**Comment:** %s\n\n' % long_desc

                if name == code:
                    name = ""
                code='**%s**' % (code)

                if reboot_required:
                    reboot_required='**Reboot required:** %s \n\n' % reboot_required
                
                scope=''
                if not len(scope_set)==1 or len(scope_set)==0:
                    scope = param.GetFieldValue("scope") or ''
                    if scope:
                        scope='**Module:** %s\n\n' % scope


                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                enum_output=''
                # Format codes and their descriptions for display. 
                if enum_codes:
                    enum_output+='**Values:**\n'
                    enum_codes=sorted(enum_codes,key=float)
                    for item in enum_codes:
                        enum_output+='\n     * **%s**: %s \n' % (item, param.GetEnumValue(item))
                    enum_output+='\n'
                    

                bitmask_list=param.GetBitmaskList() #Gets bitmask values for parameter
                bitmask_output=''
                #Format bitmask values
                if bitmask_list:
                    bitmask_output+='**Bitmask:**\n\n'
                    for bit in bitmask_list:
                        bit_text = param.GetBitmaskBit(bit)
                        bitmask_output+='    * **%s**: %s \n\n' % (bit, bit_text)
                    # bitmask_output+='\n'

                    
                # result += '%s (%s)\n\nDescription: %s %s %s %s %s %s \n\nMin > Max (Incr.): %s \n\n Default:%s \n\nUnits: %s\n\n' % (code,type,name, long_desc, enum_output, bitmask_output, reboot_required, scope, max_min_combined,def_val,unit)
                result += '* %s (%s)\n\n' % (code,type)
                result += '    %s \n\n' % (name)
                if long_desc:
                    result += '    %s' % (long_desc)
                if enum_output:
                    result += '    %s' % (enum_output)
                if bitmask_output:
                    result += '    %s' % (bitmask_output)
                if reboot_required:
                    result += '    %s' % (reboot_required)
                if scope:
                    result += '    %s' % (scope)
           
                result += '    **Min > Max (Incr.)**: %s \n\n' % (max_min_combined)
                result += '    **Default Value**: %s \n\n' % (def_val)
                result += '    **Unit**: %s \n\n' % (unit)
                

        self.output = result

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
