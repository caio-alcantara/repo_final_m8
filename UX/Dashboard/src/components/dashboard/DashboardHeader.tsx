import { useState } from "react";
import { Filter, Calendar, ChevronUp, ChevronDown } from "lucide-react";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";

const DashboardHeader = () => {
  const [isExpanded, setIsExpanded] = useState(false);

  return (
    <div className="bg-[#3C3255]">
      <div className="max-w-[1400px] mx-auto px-10">
        {/* Title row with filters toggle */}
        <div className="flex items-center justify-between py-3">
          <h1 className="text-xl font-bold text-primary-foreground">
            Acompanhamento Geral
          </h1>
          
          <button 
            onClick={() => setIsExpanded(!isExpanded)}
            className="flex items-center gap-2 text-primary-foreground/70 hover:text-primary-foreground transition-colors text-sm"
          >
            <Filter className="w-4 h-4 text-primary" />
            <span>Filtros</span>
            {isExpanded ? (
              <ChevronUp className="w-4 h-4" />
            ) : (
              <ChevronDown className="w-4 h-4" />
            )}
          </button>
        </div>
        
        {/* Collapsible filter content */}
        <div 
          className={`overflow-hidden transition-all duration-300 ease-in-out ${
            isExpanded ? "max-h-20 opacity-100 pb-3" : "max-h-0 opacity-0"
          }`}
        >
          <div className="flex items-center gap-4">
            <div className="flex flex-col">
              <span className="text-xs text-primary-foreground/70 mb-1">Data Início</span>
              <div className="flex items-center gap-2 border border-primary/30 rounded-md px-3 py-1.5 bg-[#3C3255]">
                <span className="text-sm text-primary-foreground">08/07/2025</span>
                <Calendar className="w-4 h-4 text-primary" />
              </div>
            </div>
            
            <div className="flex flex-col">
              <span className="text-xs text-primary-foreground/70 mb-1">Data Fim</span>
              <div className="flex items-center gap-2 border border-primary/30 rounded-md px-3 py-1.5 bg-[#3C3255]">
                <span className="text-sm text-primary-foreground">11/07/2025</span>
                <Calendar className="w-4 h-4 text-primary" />
              </div>
            </div>
            
            <div className="flex flex-col">
              <span className="text-xs text-primary-foreground/70 mb-1">Visualização</span>
              <Select defaultValue="dias">
                <SelectTrigger className="w-[120px] h-9 bg-[#3C3255] border-primary/30 text-primary-foreground">
                  <SelectValue placeholder="Selecione" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="dias">Dias</SelectItem>
                  <SelectItem value="semanas">Semanas</SelectItem>
                  <SelectItem value="meses">Meses</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DashboardHeader;
