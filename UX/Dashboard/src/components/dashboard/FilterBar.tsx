import { useState } from "react";
import { Filter, BarChart3, FileText, Settings, Calendar, ChevronUp, ChevronDown } from "lucide-react";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { useNavigate, useLocation } from "react-router-dom";

const FilterBar = () => {
  const [isExpanded, setIsExpanded] = useState(true);
  const navigate = useNavigate();
  const location = useLocation();

  // Função auxiliar para verificar se o botão deve estar ativo (roxo)
  const isActive = (path: string) => location.pathname === path;

  return (
    <div className="bg-[#3C3255]">
      <div className="max-w-7xl mx-auto px-8">
        {/* Toggle button - always visible */}
        <div className="flex items-center justify-between py-2">
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
          
          <div className="flex items-center gap-3">
            {/* Botão Dashboard (Home) */}
            <button 
              onClick={() => navigate("/")}
              className={`p-2 rounded-md transition-colors ${
                isActive('/') 
                  ? 'bg-primary/20 text-primary-foreground' // Estilo Ativo
                  : 'text-primary-foreground/70 hover:bg-primary/10' // Estilo Inativo
              }`}
              title="Dashboard"
            >
              <BarChart3 className={`w-5 h-5 ${isActive('/') ? 'text-primary' : 'text-current'}`} />
            </button>

            {/* Botão Documentos -> Redireciona para /documents */}
            <button 
              onClick={() => navigate("/documents")}
              className={`p-2 rounded-md transition-colors ${
                isActive('/documents') 
                  ? 'bg-primary/20 text-primary-foreground' 
                  : 'text-primary-foreground/70 hover:bg-primary/10'
              }`}
              title="Gestão de Documentos"
            >
              <FileText className={`w-5 h-5 ${isActive('/documents') ? 'text-primary' : 'text-current'}`} />
            </button>

            {/* Botão Usuários -> Redireciona para /users */}
            <button 
              onClick={() => navigate("/users")}
              className={`p-2 rounded-md transition-colors ${
                isActive('/users') 
                  ? 'bg-primary/20 text-primary-foreground' 
                  : 'text-primary-foreground/70 hover:bg-primary/10'
              }`}
              title="Gestão de Usuários"
            >
              <Settings className={`w-5 h-5 ${isActive('/users') ? 'text-primary' : 'text-current'}`} />
            </button>
          </div>
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

export default FilterBar;